"""
BLACKBOX CHALLENGE - BLE Data Retriever & Visualizer
FH Kufstein Tirol / SPS.BBM.24
pip install bleak matplotlib numpy
Usage:
    python visualizer.py
    python visualizer.py --load data.csv
"""

import asyncio, struct, argparse, time, os, csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from bleak import BleakClient, BleakScanner

SVC_UUID    = "19b10000-e8f2-537e-4f6c-d104768a1214"
STATUS_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"
META_UUID   = "19b10002-e8f2-537e-4f6c-d104768a1214"
DATA_UUID   = "19b10003-e8f2-537e-4f6c-d104768a1214"
CTRL_UUID   = "19b10004-e8f2-537e-4f6c-d104768a1214"
DEVICE_NAME = "Blackbox-1"
STATUS_IDLE=0x00; STATUS_FREEFALL=0x01; STATUS_RECORDING=0x02
STATUS_READY=0x03; STATUS_SENDING=0x04
STATUS_LABELS = {0:"IDLE",1:"FREE-FALL",2:"RECORDING",3:"DATA READY",4:"SENDING"}
CMD_REQUEST=0x01; CMD_RESET=0xFF
SAMPLE_FMT  = "<hhh"     # int16 ax, ay, az  — 6 bytes (download path only)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # 6
# BHI260AP Q12 scale: 1 g = 4096 LSB  (NOT 1000)
ACCEL_SCALE = 4096.0

# ── Data container ────────────────────────────────────────────
class BlackboxData:
    def __init__(self):
        self.ts_ms=[]; self.ax=[]; self.ay=[]; self.az=[]; self.pressure=[]
        self.total_samples=0; self.sample_rate_hz=200; self.impact_offset_ms=0
    def append_raw(self, raw):
        if len(raw) < SAMPLE_SIZE: return
        ax, ay, az = struct.unpack(SAMPLE_FMT, raw[:SAMPLE_SIZE])
        # Reconstruct timestamp from sample index + sample rate
        idx = len(self.ts_ms)
        ts  = idx * (1000 // max(self.sample_rate_hz, 1))
        self.ts_ms.append(ts); self.ax.append(ax/4096.); self.ay.append(ay/4096.)
        self.az.append(az/4096.); self.pressure.append(0.0)  # baro removed
    def resultant(self):
        return np.sqrt(np.array(self.ax)**2+np.array(self.ay)**2+np.array(self.az)**2)
    def time_axis(self):
        t=np.array(self.ts_ms,dtype=float)
        if len(t)>0: t-=t[0]
        return t
    def to_csv(self, path):
        with open(path,"w",newline="") as f:
            w=csv.writer(f)
            w.writerow(["t_ms_abs","t_ms_rel","ax_g","ay_g","az_g","resultant_g","pressure_hpa"])
            t=self.time_axis(); R=self.resultant()
            for i in range(len(self.ts_ms)):
                w.writerow([self.ts_ms[i],t[i],self.ax[i],self.ay[i],self.az[i],R[i],self.pressure[i]])
        print(f"[VIS] Saved {len(self.ts_ms)} samples -> {path}")
    @classmethod
    def from_csv(cls,path):
        d=cls()
        with open(path,newline="") as f:
            for row in csv.DictReader(f):
                d.ts_ms.append(int(float(row["t_ms_abs"]))); d.ax.append(float(row["ax_g"]))
                d.ay.append(float(row["ay_g"])); d.az.append(float(row["az_g"]))
                d.pressure.append(float(row.get("pressure_hpa", 0.0)))  # optional column
        d.total_samples=len(d.ts_ms); return d

# ── BLE retrieval ─────────────────────────────────────────────
async def scan_for_device(name, timeout=15.0):
    print(f"[BLE] Scanning for '{name}'...")
    dev = await BleakScanner.find_device_by_name(name, timeout=timeout)
    if dev is None: raise RuntimeError(f"Device '{name}' not found.")
    print(f"[BLE] Found: {dev.name}  addr={dev.address}"); return dev

async def retrieve_data(device):
    data=BlackboxData(); done=asyncio.Event()
    async with BleakClient(device, timeout=30., use_cached=False) as c:
        print(f"[BLE] Connected to {device.name}")
        st = (await c.read_gatt_char(STATUS_UUID))[0]
        print(f"[BLE] Status: {STATUS_LABELS.get(st, hex(st))}")

        # Wait for DATA_READY — polls every 2 s so you can connect at any point
        if st not in (STATUS_READY, STATUS_SENDING):
            print("[BLE] Waiting for DATA_READY (device must complete a drop)…")
            print("      Press Ctrl+C to abort.")
            while st not in (STATUS_READY, STATUS_SENDING):
                await asyncio.sleep(2.0)
                st = (await c.read_gatt_char(STATUS_UUID))[0]
                print(f"[BLE] Status: {STATUS_LABELS.get(st, hex(st))}")
        print("[BLE] DATA_READY — starting auto-download…")
        meta=await c.read_gatt_char(META_UUID)
        if len(meta)>=12:
            data.total_samples,data.sample_rate_hz,data.impact_offset_ms=struct.unpack("<III",bytes(meta[:12]))
        print(f"[BLE] {data.total_samples} samples @ {data.sample_rate_hz}Hz impact+{data.impact_offset_ms}ms")
        prog=[0]
        def on_data(ch,val):
            data.append_raw(bytes(val)); prog[0]+=1
            pct=prog[0]/max(data.total_samples,1)*100
            print(f"\r[BLE] {prog[0]}/{data.total_samples} ({pct:.0f}%)",end="",flush=True)
            if prog[0]>=data.total_samples: done.set()
        await c.start_notify(DATA_UUID, on_data)
        await c.write_gatt_char(CTRL_UUID, bytes([CMD_REQUEST]))
        try: await asyncio.wait_for(done.wait(), timeout=data.total_samples*0.02+30)
        except asyncio.TimeoutError: print(f"\n[BLE] Timeout at {prog[0]} samples")
        await c.stop_notify(DATA_UUID)
        print(f"\n[BLE] Done: {len(data.ts_ms)} samples")
    return data

# ── Phase detection ───────────────────────────────────────────
PHASE_IDLE=0; PHASE_FREEFALL=1; PHASE_IMPACT=2; PHASE_SETTLE=3
PHASE_COLORS={0:("#1f6feb","IDLE",0.12),1:("#388bfd","FREE-FALL",0.18),
              2:("#f85149","IMPACT",0.25),3:("#3fb950","SETTLE",0.12)}

def detect_phases(t, R, impact_offset_ms=0):
    """
    Improved phase detection — searches backward from the impact peak
    so long live-stream recordings don't produce bogus 8000ms free-falls.

    Steps:
      1. Find the primary impact peak (max R, or use impact_offset_ms hint).
      2. Walk BACKWARD from the peak to find where gravity last returned
         above 0.60g — that is the true free-fall start.
      3. Walk FORWARD from the peak to find where R drops back below 2g
         — that is settle start.
    """
    n=len(R)
    if n==0: return np.array([],dtype=int),0.,0.,0.,0.
    phases=np.full(n,PHASE_IDLE,dtype=int)

    # ── 1. Find primary impact index ─────────────────────────────
    imp=int(np.argmax(R))
    if R[imp]<3.0:                   # no real impact in this dataset
        return phases,0.,0.,0.,float(t[-1])

    # Allow firmware-provided hint to override if close to peak
    if impact_offset_ms>0:
        hint=int(np.argmin(np.abs(t-(t[0]+impact_offset_ms))))
        if abs(t[hint]-t[imp])<2000: imp=hint

    # ── 2. Free-fall start: walk backward from BEFORE the impact ─────
    # Start at imp-1 so the high-G impact sample doesn't stop us immediately
    ffi=imp
    for i in range(imp-1,-1,-1):
        if R[i]>0.60:
            ffi=i; break   # ffi = last IDLE-level sample before drop
    # Verify the region [ffi:imp] actually contained real free-fall (<0.30g)
    if ffi<imp and np.min(R[ffi:imp])>0.30:
        ffi=imp   # nothing low-G found → no free-fall, start == impact

    # ── 3. Settle start: walk forward until R drops below 2g ─────
    setl=min(imp+max(int((n-imp)*0.25),5),n-1)
    for i in range(imp,n):
        if R[i]<2.0 and i>imp+2:
            setl=i; break

    # ── 4. Build phase array ──────────────────────────────────────
    if ffi<imp:
        phases[ffi:imp]=PHASE_FREEFALL
    phases[imp:setl]=PHASE_IMPACT
    phases[setl:]=PHASE_SETTLE

    def _t(i): return float(t[i]) if 0<=i<n else 0.
    return phases,_t(ffi),_t(imp),_t(imp),_t(setl)

# ── 7-panel drop analysis dashboard ──────────────────────────
def plot_drop_data(data, title="Blackbox Impact Record"):
    t=data.time_axis(); Ax=np.array(data.ax); Ay=np.array(data.ay)
    Az=np.array(data.az); R=data.resultant(); P=np.array(data.pressure)
    if len(t)==0: print("[VIS] No data."); return None
    n=len(t); tspan=float(t[-1]-t[0])
    jerk=np.gradient(R, t/1000.)
    tilt=np.degrees(np.arctan2(np.sqrt(Ax**2+Ay**2),np.abs(Az)))
    phases,ff0,ff1,imp_ms,setl_ms=detect_phases(t,R,data.impact_offset_ms)
    ff_dur=(ff1-ff0) if ff1>ff0 else 0.
    ff_s=ff_dur/1000.
    v_imp=9.81*ff_s; h_est=0.5*9.81*ff_s**2
    peakG=float(np.max(R)); peakGt=float(t[int(np.argmax(R))])
    peakJ=float(np.max(np.abs(jerk)))
    imask=R>4.0; imp_dur=float(np.sum(imask))*(tspan/n) if np.any(imask) else 0.
    pvalid=bool(np.any(P>0))
    pdrop=float(np.max(P)-np.min(P)) if pvalid else 0.
    alt_drop=pdrop/0.012
    tilt_imp=float(tilt[int(np.argmin(np.abs(t-imp_ms)))]) if imp_ms>0 else float(tilt[n//2])
    # FFT on post-settle
    fft_freq=fft_mag=None; dom_hz=0.
    si=int(np.argmin(np.abs(t-setl_ms))) if setl_ms<t[-1] else n//2
    postR=R[si:] if (n-si)>8 else R[n//2:]
    if len(postR)>8:
        sr=n/(tspan/1000.) if tspan>0 else 200.
        fv=np.abs(np.fft.rfft(postR-np.mean(postR)))
        fft_freq=np.fft.rfftfreq(len(postR),d=1./sr); fft_mag=fv
        if len(fft_freq)>1: dom_hz=float(fft_freq[int(np.argmax(fft_mag[1:]))+1])
    # colours
    BG="#0d1117";ABG="#161b22";GR="#30363d";TC="#c9d1d9";DIM="#8b949e"
    CX="#79c0ff";CY="#56d364";CZ="#ffa657";CR="#f78166"
    CJ="#d2a8ff";CTL="#e3b341";CP="#58a6ff";CIMP="#ff6b6b"
    fig=plt.figure(figsize=(18,16),facecolor=BG)
    fig.suptitle("BLACKBOX DROP ANALYSIS  —  "+title,fontsize=12,color="white",
                 fontweight="bold",y=0.997,x=0.5)
    gs=gridspec.GridSpec(5,3,figure=fig,height_ratios=[0.32,1.9,1.9,1.7,2.1],
                         hspace=0.60,wspace=0.36,left=0.07,right=0.97,top=0.962,bottom=0.06)
    def sty(a,yl="",xl="Time (ms)"):
        a.set_facecolor(ABG); a.tick_params(colors=TC,labelsize=8)
        a.set_xlabel(xl,color=DIM,fontsize=8); a.set_ylabel(yl,color=DIM,fontsize=8)
        a.grid(True,color=GR,linewidth=0.4,alpha=0.8); a.set_xlim(t[0],t[-1])
        for sp in a.spines.values(): sp.set_edgecolor(GR)
    def shade(a):
        segs=[(t[0],ff0,0),(ff0,imp_ms if imp_ms else t[-1],1),(imp_ms,setl_ms,2),(setl_ms,t[-1],3)]
        for lo,hi,ph in segs:
            lo=max(float(lo),float(t[0])); hi=min(float(hi),float(t[-1]))
            if hi>lo:
                col,_,alpha=PHASE_COLORS[ph]; a.axvspan(lo,hi,alpha=alpha,color=col,zorder=0,linewidth=0)
    def vl(a,x=None,lbl="",col=CIMP):
        x=x if x is not None else imp_ms
        if x and x>t[0]: a.axvline(x=x,color=col,linewidth=1.1,linestyle="--",alpha=0.75,label=lbl,zorder=5)
    # ── Panel 0: phase timeline
    ab=fig.add_subplot(gs[0,:]); ab.set_facecolor(BG)
    for sp in ab.spines.values(): sp.set_visible(False)
    ab.set_xlim(t[0],t[-1]); ab.set_ylim(0,1); ab.set_yticks([])
    ab.tick_params(colors=DIM,labelsize=8,length=3)
    segs=[(t[0],ff0,0),(ff0,imp_ms if imp_ms else t[-1],1),(imp_ms,setl_ms,2),(setl_ms,t[-1],3)]
    total_span=float(t[-1]-t[0]) if t[-1]>t[0] else 1.
    for lo,hi,ph in segs:
        lo=float(lo); hi=float(hi)
        if hi>lo:
            col,lbl,_=PHASE_COLORS[ph]; ab.axvspan(lo,hi,ymin=0,ymax=1,color=col,alpha=0.85)
            # Only draw text if segment is at least 4% of total span (avoids overlap on short drops)
            if (hi-lo)/total_span > 0.04:
                ab.text((lo+hi)/2,0.5,lbl,ha="center",va="center",color="white",fontsize=8,
                        fontweight="bold",transform=ab.get_xaxis_transform())
    if ff_dur>0 and imp_ms>0:
        ab.annotate(f"{ff_dur:.0f} ms",xy=((ff0+imp_ms)/2,0.88),color=TC,fontsize=7,
                    ha="center",xycoords=("data","axes fraction"))
    ab.set_title("Drop Phase Timeline",color=TC,fontsize=9,pad=3,loc="left")
    ab.set_xlabel("Time (ms)",color=DIM,fontsize=8)
    # ── Panel 1: X/Y/Z
    a1=fig.add_subplot(gs[1,:]); shade(a1)
    a1.plot(t,Ax,color=CX,linewidth=0.85,label="X",zorder=3)
    a1.plot(t,Ay,color=CY,linewidth=0.85,label="Y",zorder=3)
    a1.plot(t,Az,color=CZ,linewidth=0.85,label="Z",zorder=3)
    a1.axhline(y=0,color=GR,linewidth=0.4,linestyle=":",zorder=2)
    a1.axhline(y=16,color=CIMP,linewidth=0.5,linestyle=":",alpha=0.4)
    a1.axhline(y=-16,color=CIMP,linewidth=0.5,linestyle=":",alpha=0.4)
    vl(a1,imp_ms,"Impact")
    if ff_dur>5 and imp_ms>0:
        yl=a1.get_ylim()
        a1.text((ff0+imp_ms)/2,yl[1]*0.82 if yl[1]>0 else 0.5,
                f"  free-fall  {ff_dur:.0f} ms",ha="center",color=CX,fontsize=8,style="italic")
    a1.legend(loc="upper right",facecolor=ABG,labelcolor=TC,fontsize=8,framealpha=0.8,ncol=3)
    a1.set_title("Acceleration  X / Y / Z  (g)",color=TC,fontsize=11,pad=4)
    a1.text(t[-1]*0.99 if t[-1]>0 else 1,15,"sensor +-16g limit",
            ha="right",color=CIMP,fontsize=6.5,alpha=0.5)
    sty(a1,"g")
    # ── Panel 2: resultant
    a2=fig.add_subplot(gs[2,:]); shade(a2)
    a2.plot(t,R,color=CR,linewidth=1.1,label="|g| resultant",zorder=3)
    a2.fill_between(t,0,R,alpha=0.12,color=CR,zorder=2)
    a2.axhline(y=1.0,color=CY,linewidth=0.7,linestyle=":",alpha=0.7,label="1g stationary")
    a2.axhline(y=0.25,color=CX,linewidth=0.7,linestyle=":",alpha=0.7,label="0.25g freefall thresh")
    a2.axhline(y=4.0,color=CZ,linewidth=0.7,linestyle=":",alpha=0.7,label="4g impact thresh")
    vl(a2,imp_ms,"Impact")
    if peakG>0:
        a2.annotate(f"  Peak: {peakG:.1f} g",xy=(peakGt,peakG),
                    xytext=(peakGt+tspan*0.02,peakG*0.88),color=CR,fontsize=9,fontweight="bold",
                    arrowprops=dict(arrowstyle="->",color=CR,lw=0.9))
    if imp_dur>0 and np.any(imask):
        ilo=float(t[imask][0]); ihi=float(t[imask][-1])
        a2.annotate("",xy=(ihi,4.3),xytext=(ilo,4.3),
                    arrowprops=dict(arrowstyle="<->",color=CZ,lw=1.0))
        a2.text((ilo+ihi)/2,4.6,f"{imp_dur:.0f} ms above 4g",ha="center",color=CZ,fontsize=7)
    a2.legend(loc="upper right",facecolor=ABG,labelcolor=TC,fontsize=8,framealpha=0.8,ncol=2)
    a2.set_title("Resultant Acceleration  |g|",color=TC,fontsize=11,pad=4)
    sty(a2,"|g| (g)")
    # ── Panel 3: jerk
    a3=fig.add_subplot(gs[3,0]); shade(a3)
    a3.plot(t,jerk,color=CJ,linewidth=0.75,label="Jerk",zorder=3)
    a3.fill_between(t,0,jerk,where=(jerk>0),alpha=0.14,color=CJ)
    a3.fill_between(t,0,jerk,where=(jerk<0),alpha=0.10,color=CIMP)
    a3.axhline(y=0,color=GR,linewidth=0.4,linestyle=":"); vl(a3,imp_ms)
    a3.set_title(f"Jerk  d|g|/dt  (peak {peakJ:.0f} g/s)",color=TC,fontsize=9,pad=4)
    a3.legend(loc="upper right",facecolor=ABG,labelcolor=TC,fontsize=7,framealpha=0.8)
    sty(a3,"g/s")
    # ── Panel 4: tilt
    a4=fig.add_subplot(gs[3,1]); shade(a4)
    a4.plot(t,tilt,color=CTL,linewidth=0.75,label="Tilt",zorder=3)
    for yy in [0,90,180]: a4.axhline(y=yy,color=GR,linewidth=0.4,linestyle=":",alpha=0.5)
    vl(a4,imp_ms); a4.set_ylim(-5,185); a4.set_yticks([0,45,90,135,180])
    a4.set_title(f"Tilt Angle  (at impact: {tilt_imp:.1f} deg)",color=TC,fontsize=9,pad=4)
    a4.text(0.01,0.10,"0=flat  90=vertical  180=upside-down",transform=a4.transAxes,color=DIM,fontsize=6.5)
    a4.legend(loc="upper right",facecolor=ABG,labelcolor=TC,fontsize=7,framealpha=0.8)
    sty(a4,"degrees")
    # ── Panel 5: cumulative velocity estimate (replaces pressure — irrelevant for 3m drop)
    a5=fig.add_subplot(gs[3,2]); shade(a5)
    # Integrate |g|-1 (net decel) over free-fall window to show velocity build-up
    dt_s=(t[1]-t[0])/1000. if len(t)>1 else 0.005
    net_a=(R-1.0)*9.81    # net acceleration m/s² (subtract gravity)
    vel=np.cumsum(net_a)*dt_s
    a5.plot(t,vel,color=CP,linewidth=0.9,label="Est. velocity (m/s)",zorder=3)
    a5.fill_between(t,0,vel,alpha=0.12,color=CP)
    a5.axhline(y=0,color=GR,linewidth=0.4,linestyle=":")
    if v_imp>0:
        a5.axhline(y=v_imp,color=CY,linewidth=0.7,linestyle="--",alpha=0.7,
                   label=f"v_impact = {v_imp:.2f} m/s")
    vl(a5,imp_ms)
    a5.set_title(f"Velocity Estimate  (v_impact ≈ {v_imp:.2f} m/s)",color=TC,fontsize=9,pad=4)
    a5.legend(loc="upper right",facecolor=ABG,labelcolor=TC,fontsize=7,framealpha=0.8)
    sty(a5,"m/s")
    # ── Panel 6: FFT
    a6=fig.add_subplot(gs[4,0]); a6.set_facecolor(ABG)
    a6.tick_params(colors=TC,labelsize=8); a6.set_xlabel("Frequency (Hz)",color=DIM,fontsize=8)
    a6.set_ylabel("Magnitude",color=DIM,fontsize=8); a6.grid(True,color=GR,linewidth=0.4,alpha=0.8)
    for sp in a6.spines.values(): sp.set_edgecolor(GR)
    if fft_freq is not None and len(fft_freq)>1:
        a6.plot(fft_freq[1:],fft_mag[1:],color=CJ,linewidth=0.8)
        a6.fill_between(fft_freq[1:],0,fft_mag[1:],alpha=0.15,color=CJ)
        if dom_hz>0:
            pkm=float(fft_mag[int(np.argmax(fft_mag[1:]))+1])
            a6.axvline(x=dom_hz,color=CZ,linewidth=1.0,linestyle="--",alpha=0.8,label=f"Dom: {dom_hz:.1f} Hz")
            a6.annotate(f"{dom_hz:.1f} Hz",xy=(dom_hz,pkm),
                        xytext=(dom_hz+(fft_freq[-1]-fft_freq[0])*0.04,pkm*0.82),color=CZ,fontsize=8,
                        arrowprops=dict(arrowstyle="->",color=CZ,lw=0.7))
        a6.legend(loc="upper right",facecolor=ABG,labelcolor=TC,fontsize=7,framealpha=0.8)
    else:
        a6.text(0.5,0.5,"Insufficient post-impact\nsamples for FFT",ha="center",va="center",
                color=DIM,transform=a6.transAxes,fontsize=8)
    a6.set_title("Post-Impact Vibration Spectrum (FFT)",color=TC,fontsize=9,pad=4)
    # ── Panel 7: stats  (2-column layout to avoid overlap)
    a7=fig.add_subplot(gs[4,1:]); a7.set_facecolor(ABG); a7.axis("off")
    for sp in a7.spines.values(): sp.set_edgecolor(GR)
    sr_act=n/(tspan/1000.) if tspan>0 else 0.
    # Left column
    left=[("RECORDING",""),
          ("  Samples",  f"{n}"),
          ("  Duration",  f"{tspan:.0f} ms"),
          ("  Sample rate",f"{sr_act:.0f} Hz"),
          ("",""),
          ("FREE-FALL",""),
          ("  Duration",   f"{ff_dur:.0f} ms"),
          ("  Height est.",f"{h_est:.2f} m"),
          ("  Impact vel.",f"{v_imp:.2f} m/s")]
    # Right column
    right=[("IMPACT",""),
           ("  Peak |g|",  f"{peakG:.2f} g"),
           ("  Peak @ t",  f"{peakGt:.0f} ms"),
           ("  Dur > 4g",  f"{imp_dur:.1f} ms"),
           ("  Peak jerk", f"{peakJ:.0f} g/s"),
           ("  Tilt",      f"{tilt_imp:.1f} deg"),
           ("",""),
           ("POST-IMPACT",""),
           ("  Dom. vib.",  f"{dom_hz:.1f} Hz" if dom_hz>0 else "--"),
           ("  Est. energy", f"{0.5*(v_imp**2):.2f} J/kg")]
    HDR="#58a6ff"
    def draw_col(rows, x_lbl, x_val, y_start=0.95, dy=0.088):
        y=y_start
        for lbl,val in rows:
            if lbl=="":
                y-=dy*0.4; continue
            is_hdr=(val=="")
            color=HDR if is_hdr else DIM
            fw="bold" if is_hdr else "normal"
            fs=8.5 if is_hdr else 8.0
            a7.text(x_lbl,y,lbl,transform=a7.transAxes,color=color,
                    fontsize=fs,fontweight=fw,va="top",family="monospace")
            if not is_hdr:
                a7.text(x_val,y,val,transform=a7.transAxes,color=TC,
                        fontsize=8.0,fontweight="bold",va="top",family="monospace")
            y-=dy
    draw_col(left,  0.02, 0.28)
    draw_col(right, 0.52, 0.78)
    a7.set_title("Drop Analytics Summary",color=TC,fontsize=9,pad=4,loc="left")
    fig.patch.set_facecolor(BG)
    return fig

# ── BLE main ──────────────────────────────────────────────────
async def main_ble(args):
    dev=await scan_for_device(DEVICE_NAME); data=await retrieve_data(dev)
    if not data.ts_ms: print("[VIS] No data."); return
    ts=time.strftime("%Y%m%d_%H%M%S")
    csv_out=os.path.join(os.path.dirname(__file__),f"blackbox_drop_{ts}.csv")
    data.to_csv(csv_out)
    fig=plot_drop_data(data,title=f"Drop Log -- {ts}")
    if fig:
        png_out=csv_out.replace(".csv",".png")
        fig.savefig(png_out,dpi=150,bbox_inches="tight",facecolor=fig.get_facecolor())
        print(f"[VIS] Plot -> {png_out}"); plt.show()

def main_csv(path):
    print(f"[VIS] Loading {path}...")
def main_csv(path):
    print(f"[VIS] Loading {path}...")
    data=BlackboxData.from_csv(path)
    fig=plot_drop_data(data,title=f"Drop Log -- {os.path.basename(path)}")
    if fig: plt.show()

if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("--load",metavar="CSV")
    args=p.parse_args()
    if args.load: main_csv(args.load)
    else: asyncio.run(main_ble(args))
