# Mechanism Analysis – Continuum Leg Simulator

Publication-ready simulator for the TPU flexible leg (d 7.0 v.1.1) with CPG gait integration.

## Files

| File | Description |
|------|-------------|
| `continuum_simulator_pro.py` | Main professional simulator: visualization, CPG modes, animation export |
| `cpg_continuum.py` | Python CPG model for one leg: Trot, Crawl, Creep gaits, mapped to tendon lengths |
| `simulaterKinematicModel.py` | Original interactive model (sliders only) |

## Quick Start

```bash
cd Mechanism_Analysis
pip install -r requirements.txt
python continuum_simulator_pro.py
```

## Features

### Professional Visualization
- High-DPI rendering (150 DPI default, 300 DPI for screenshots)
- Serif fonts and clear layout suitable for papers
- Segmented leg geometry with square cross-section and V-shaped grooves

### CPG Integration
- **Manual**: Use sliders for l1–l4
- **Trot**: Diagonal-sync gait (Trot / Turtle)
- **Crawl**: Diagonal in-phase gait
- **Creep**: Lateral sequence, 4-beat gait

CPG dynamics are ported from `control_1.8/main/cpg.c` and `gate.c`. Hip and knee outputs are mapped to tendon lengths (l1–l4) for the continuum leg.

### Screenshots
- **Screenshot** button: saves `continuum_leg_screenshot.png` at 300 DPI

### Animation Export
- **Export Animation** button: saves GIF (and MP4 if ffmpeg is available) for the current gait
- Produces `continuum_leg_trot.gif`, `continuum_leg_crawl.gif`, `continuum_leg_creep.gif`

## CPG-to-Continuum Mapping

The CPG outputs hip and knee joint targets (encoder counts). For the continuum leg:

- **Hip** → bending direction (φ) and amount (θ)
- **Knee** → arc length (s)

These are converted to tendon lengths:

```
s = (l1+l2+l3+l4)/4
dl_x = 2*d*θ*cos(φ),  dl_y = 2*d*θ*sin(φ)
l1 = s - dl_x/2,  l3 = s + dl_x/2
l2 = s - dl_y/2,  l4 = s + dl_y/2
```

## Dependencies

- numpy
- matplotlib
- Pillow (for GIF export)
- ffmpeg (optional, for MP4 export)
