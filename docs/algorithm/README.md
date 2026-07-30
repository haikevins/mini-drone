# Complementary Filter

## 1. Ý tưởng cốt lõi của Complementary Filter

- Hai cảm biến có điểm mạnh/yếu khác nhau:

  - **Gyroscope (`gx`, `gy`, `gz`)**: Đo tốc độ góc (`deg/s`), khi tích phân theo thời gian → cho góc.
    - Ưu điểm: Rất mượt, phản ứng nhanh.
    - Nhược điểm: Bị drift (trôi sai số theo thời gian).

  - **Accelerometer (`ax`, `ay`, `az`)**: Đo hướng trọng lực, suy ra góc roll/pitch trực tiếp.
    - Ưu điểm: Không bị drift lâu dài.
    - Nhược điểm: Rất nhiễu khi rung/di chuyển nhanh.

- Gyro: tốt ở high frequency (ngắn hạn).
- Accel: tốt ở low frequency (dài hạn).

→ Ghép lại thành một hệ ổn định toàn dải tần.

## 2. Ý tưởng “bổ sung”

Complementary Filter kết hợp:

- Gyro → tốt cho ngắn hạn.
- Accelerometer → tốt cho dài hạn.

Công thức tổng quát:

```text
θ = α(θgyro) + (1 − α)(θacc)
```

Trong đó:

- `α` (alpha) gần 1 → tin gyro nhiều hơn.
- `1 - α` → tin accelerometer.

## 3. Luồng hoạt động

### Bước 1: Tích phân gyro (dự đoán nhanh)

```c
gyroRoll = attitude.roll + gx * dt;
gyroPitch = attitude.pitch + gy * dt;
gyroYaw = attitude.yaw + gz * dt;
```

Dạng liên tục (continuous time):

```text
θ(t) = θ(0) + ∫ 0→t (ω(τ)dτ)
```

Trong đó:

- `θ(t)`: góc tại thời điểm `t`.
- `ω(t)`: tốc độ góc (gyro).
- `ω = (gx, gy, gz)`.

Dạng rời rạc: Vì chạy theo từng bước thời gian `dt`, ta có:

```text
θk+1 = θk + ωk.Δt
```

Viết riêng trên từng trục:

```text
Roll:  ϕk+1 = ϕk + gx.Δt
Pitch: θk+1 = θk + gy.Δt
Yaw:   ψk+1 = ψk + gz.Δt
```

Ý nghĩa:

- Gyro đo: tốc độ thay đổi góc.
- Nhân `dt`: ra độ thay đổi góc.
- Cộng vào góc cũ: ra góc mới.

### Bước 2: Tính góc từ accelerometer

```text
rollAcc = atan2(ay, az)
pitchAcc = atan2(ax, sqrt(ay² + az²))
```

### Bước 3: Trộn 2 nguồn

```c
attitude.roll =
    alpha * gyroRoll + (1 - alpha) * rollAcc;

attitude.pitch =
    alpha * gyroPitch + (1 - alpha) * pitchAcc;
```

Đây là complementary được áp dụng.

### Bước 4: Yaw

```c
attitude.yaw = gyroYaw;
```

Không có magnetometer nên không tính được `yawAccel` → sẽ bị drift theo thời gian do gyro.
