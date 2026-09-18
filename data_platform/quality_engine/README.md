# Data Quality Engine
**Owner: Sireesha**

## Validation Pipeline
Before raw data is cataloged or used for training:
1. **Integrity Check:** File format, header consistency, checksum validation.
2. **Timestamp & Sync Check:** Multi-camera, LiDAR, and radar timestamp alignment ($< 10\text{ ms}$ jitter).
3. **Sensor Calibration:** Extrinsic and intrinsic matrix verification.
4. **Outlier Detection:** Anomaly filtering for coordinate outliers and corrupted point clouds.
5. **Leakage Prevention:** Sequence-level dataset splitting (Train sequences $\ne$ Test sequences).
