# Hardware Power Requirements & Deployment Bundle

## 1. Executive Summary
**Critical Issue Identified**: The system requires a robust Power over Ethernet (PoE) budget to stably operate 4 Luxonis OAK cameras with Neural Networks enabled.
**Finding**: Standard PoE "Passthrough" switches (powered by an upstream PoE line) cannot supply sufficient current for 4 cameras during the NN initialization spike, causing device dropouts.
**Recommendation**: Must use a **dedicated, mains-powered PoE switch** with at least **60W** total budget.

## 2. Power Specifications

### System Load
| Component | Device | Quantity | Typical Power | Peak Power (Startup/NN) | Total Peak Load |
|-----------|--------|----------|---------------|-------------------------|-----------------|
| Camera | Luxonis OAK-D PoE | 4 | ~6W each (24W) | ~12W each | **~48W** |

### Failure Scenario (Avoid)
- **Hardware**: Intellinet 5-Port Gigabit Switch (PoE Powered / Passthrough).
- **Mechanism**: When powered by standard PoE+ (30W input), the available budget for downstream devices is often capped at ~25W.
- **Result**: System crash / `ping missed` errors when initializing the 3rd or 4th camera.

### Verified Solution (Recommended)
- **Hardware**: Ubiquiti UniFi Switch 8 60W (US-8-60W) or equivalent.
- **Spec**:
  - IEEE 802.3af compliant.
  - **60W Total Available PoE Budget**.
  - **15.4W Max per port**.
- **Result**: Stable operation of all 4 cameras at 5 FPS with NN enabled.

## 3. Deployment Checklist

### Hardware
- [ ] **Power**: Ensure the PoE switch accepts mains power (AC), not just upstream PoE.
- [ ] **Cabling**: Connect cameras directly to the PoE switch ports (Ports 5-8 on US-8-60W).
- [ ] **Thermal**: Ensure cameras have airflow; NN inference generates significant heat.

### Software Configuration
The `sea_surface_segmentation` node includes stability features to handle power/connection timing:
- **Retry Logic**: The node attempts connection 5 times with a 2-second delay to allow for staggered power-up.
- **Configurable Load**:
  - `fps`: Lowering FPS (e.g., to 2.0 or 5.0) reduces continuous power draw manifestions.
  - `enable_nn`: Can be disabled for pure video streaming if power budget is strictly limited.
