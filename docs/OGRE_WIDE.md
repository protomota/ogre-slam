# OGRE_WIDE.md

**Alternative Wide-Base Robot Configuration**

This document describes an experimental wider robot configuration (`ogre_stable.usd`) that was developed to improve stability in Isaac Sim. The project has since returned to the original robot dimensions, but this configuration remains available for reference and future experimentation.

---

## Robot Physical Dimensions (ogre_stable.usd)

### Body (Main Chassis)
- **Length (X):** 0.20m (200mm)
- **Width (Y):** 0.16m (160mm)
- **Height (Z):** 0.110m (110mm) - **37% LOWER** than original (175mm)
- **Position:** 20mm above wheel axle (body bottom at Z=0.06m, center at Z=0.075m)
- **Mass:** ~2.5kg (no barrels/battery)

### Overall Robot Footprint
- **Total Length:** ~280mm (wheelbase + wheel width)
- **Total Width:** 340mm (track width) - **66% WIDER** than original (205mm)
- **Total Height:** ~265mm (to top of LIDAR)
- **Total Mass:** ~3.5kg (lighter, no top-heavy components)

### Wheels
- **Radius:** 0.040m (40mm)
- **Width:** 0.040m (40mm)
- **Wheelbase:** 0.200m (200mm front-to-rear) - **111% LONGER** than original (95mm)
- **Track width:** 0.340m (340mm left-to-right) - **66% WIDER** than original (205mm)
- **Mass:** ~0.1kg each

### Wheel Positions (relative to base_link at wheel axle height)
- **Front-Left (FL):** (0.1, 0.17, 0.04)
- **Front-Right (FR):** (0.1, -0.17, 0.04)
- **Rear-Left (RL):** (-0.1, 0.17, 0.04)
- **Rear-Right (RR):** (-0.1, -0.17, 0.04)

### LIDAR Mounting
- Posts adjusted for shorter body
- LIDAR frame at Z=0.195m (lower due to shorter body)
- Alternative: Z=0.30m with taller posts (original height)
- Rotated 180° around Z axis

### Camera Mounting
- Isaac Sim: `front_camera` frame at (0.15, 0, 0.10) - 15cm forward, 10cm above base_link
- Real Robot: `camera_link` for RealSense D435 at same position (0.15, 0, 0.10)

---

## Design Goals

This configuration was designed to address physics instability in Isaac Sim:

### Stability Improvements
- ✅ **Center of gravity 49% lower** (147.5mm → 75mm)
- ✅ **111% longer wheelbase** (95mm → 200mm) for front/back stability
- ✅ **66% wider track** (205mm → 340mm) for side-to-side stability
- ✅ **No top-heavy components** (removed barrels and battery)
- ✅ **22% lighter overall mass** (4.5kg → 3.5kg)

### Trade-offs
- ❌ **Wider footprint** - requires larger navigation spaces
- ❌ **Different from real robot** - simulation doesn't match hardware
- ❌ **Maze needs larger cells** - 80cm cells vs 60cm for original

---

## Maze Configuration for Wide Robot

The wider robot requires larger maze cells:

**Maze Settings for ogre_stable.usd:**
- **Cell size:** 80cm × 80cm (vs 60cm for original)
- **Robot diagonal:** 390mm (340mm track × 200mm wheelbase)
- **Clearance:** 410mm diagonal clearance (comfortable)
- **Maze dimensions:** 4.0m × 4.0m (5×5 cells)
- **Position:** Centered at (-2.0, -2.0, 0.6)

---

## Isaac Sim Physics Settings

These settings were found to work well for the wide robot:

### Wheel Joint Settings (RevoluteJoint)
- **Maximum Velocity:** 10000 (critical for stability!)
- **Damping:** 1.0-10.0
- **Stiffness:** 0
- **Max Force:** 100-1000

### Physics Scene Settings
- **Position Iteration Count:** 8
- **Velocity Iteration Count:** 2
- **Time Steps Per Second:** 120+
- **Enable CCD:** True

### Collision Configuration
- **Wheel collision:** Simple approximation (boundingCube or boundingSphere)
- **Roller collision:** Disabled (28 rollers total)
- **No overlaps:** Verified with collision visualization

---

## Comparison: Original vs Wide

| Aspect | Original (ogre.usd) | Wide (ogre_stable.usd) |
|--------|---------------------|------------------------|
| **Body Height** | 175mm | 110mm (-37%) |
| **Wheelbase** | 95mm | 200mm (+111%) |
| **Track Width** | 205mm | 340mm (+66%) |
| **Total Mass** | 4.5kg | 3.5kg (-22%) |
| **Barrels/Battery** | Yes (0.71kg each) | No (removed) |
| **CoG Height** | 147.5mm | 75mm (-49%) |
| **Maze Cell Size** | 60cm | 80cm |
| **Matches Real Robot** | ✅ Yes | ❌ No |

---

## Status

**Current:** Project uses original robot (ogre.usd) to match real hardware

**Future:** This wide configuration remains available in `ogre_stable.usd` for:
- Testing stability algorithms
- Experimenting with different robot geometries
- Comparing navigation performance with wider wheelbase

---

## Files

- **Robot Model:** `/home/brad/ros2_ws/src/ogre-slam/ogre_stable.usd`
- **Documentation:** This file (OGRE_WIDE.md)
- **Primary Robot:** See CLAUDE.md for current ogre.usd specifications
