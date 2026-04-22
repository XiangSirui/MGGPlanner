# Communication Model Reference Baseline

This project's communication model in `mggplanner/src/comm_link_model.cpp` follows a
log-distance + map-obstruction + shadowing + SNR budget structure.

## 1) Equations and where they come from

1. **Large-scale path loss (log-distance model)**

   \[
   PL(d)=PL_0(d_0)+10n\log_{10}(d/d_0)
   \]

   - Reference:
     - A. Goldsmith, *Wireless Communications*, Cambridge Univ. Press, 2005.
     - T. S. Rappaport, *Wireless Communications: Principles and Practice*, 2nd ed., 2002.

2. **Reference path loss from Friis (when `comm_link_carrier_freq_hz > 0`)**

   \[
   PL_0(d_0)=20\log_{10}\left(\frac{4\pi d_0 f}{c}\right)-G_t-G_r+L_{misc}
   \]

   - Implemented in code as automatic PL0 derivation.
   - Reference:
     - A. Goldsmith, 2005 (Friis/link budget background).
     - T. S. Rappaport, 2002.

3. **Thermal noise floor (when `comm_link_bandwidth_hz > 0`)**

   \[
   N_{dBm}=-174+10\log_{10}(B_{Hz})+NF
   \]

   - Reference:
     - A. Goldsmith, 2005 (receiver noise and link budgets).

4. **Log-normal shadowing**

   \[
   PL_{total}=PL(d)+X_\sigma,\ \ X_\sigma\sim\mathcal{N}(0,\sigma^2)
   \]

   - In this codebase, `X_sigma` is deterministic per link endpoint hash for
     reproducible planning/simulation.
   - Reference:
     - A. Goldsmith, 2005.
     - T. S. Rappaport, 2002.

5. **Map-coupled LOS/NLOS effects**
   - The planner estimates blocked/unknown ratios via multi-ray queries on the occupancy map.
   - This is a pragmatic robotics adaptation consistent with:
     - D. A. Shell and M. J. Mataric, "High-Fidelity Radio Communications Modeling for Multi-Robot Simulation," IROS 2009.

## 2) Baseline parameter set (used in SMB configs)

Configured in:
- `mggplanner/config/smb/mggplanner_config_sim.yaml`
- `mggplanner/config/smb/mgg_darpa.yaml`

Baseline values:

- `comm_link_carrier_freq_hz: 2.4e9`
- `comm_link_d0_m: 1.0`
- `comm_link_tx_antenna_gain_dbi: 0.0`
- `comm_link_rx_antenna_gain_dbi: 0.0`
- `comm_link_misc_system_loss_db: 0.0`
- `comm_link_path_loss_exp_los: 2.0`
- `comm_link_path_loss_exp_nlos: 3.5`
- `comm_link_bandwidth_hz: 20e6`
- `comm_link_noise_figure_db: 6.0`
- `comm_link_shadowing_std_db: 6.0`
- `comm_link_geom_los_obstructed_loss_db: 15.0`
- `comm_link_min_snr_db: 10.0`

Interpretation:
- LOS/NLOS exponents and shadowing sigma are in typical ranges reported by
  mainstream wireless channel references and should still be calibrated by your
  own logs for your environment.
- `comm_link_geom_los_obstructed_loss_db` is a prior for map-obstructed direct
  ray and should be validated with field data.

## 3) Calibration requirement (for paper-grade credibility)

The above is a literature-grounded baseline, not final truth for every site.
For publication-quality claims, fit these parameters from your own dataset:

- `comm_link_path_loss_exp_los`
- `comm_link_path_loss_exp_nlos`
- `comm_link_shadowing_std_db`
- `comm_link_geom_los_obstructed_loss_db`
- `comm_link_min_snr_db` (from target PDR curve)

Recommended reporting:
- RMSE/MAE of predicted vs measured RSSI/SNR.
- PDR-vs-SNR curve and chosen operating threshold.

## 4) Quick links

- Goldsmith book page: <https://www.cambridge.org/core/books/wireless-communications/800C5AAFA8CC75A7809597C2F67F8D87>
- Rappaport book (2nd ed.): <https://www.pearson.com/en-us/subject-catalog/p/wireless-communications-principles-and-practice/P200000003472/9780130422323>
- Shell & Mataric IROS 2009 PDF: <https://cse-robotics.engr.tamu.edu/dshell/papers/iros2009radio.pdf>
