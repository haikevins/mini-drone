# Note

## 1. Hệ trục X-quad

```text
M1 = front-left
M2 = front-right
M3 = rear-right
M4 = rear-left
```

## 2. Cánh test

```text
M1 front-left  = CW
M2 front-right = CCW
M3 rear-right  = CW
M4 rear-left   = CCW
```

## 3. IMU và Madgwick test

### Level / +Z up

```text
ax=0.01 ay=-0.02 az=1.00
```

→ đúng, gần `(0, 0, +1)`.

### Upside down / -Z up

```text
ax=-0.03 ay=0.01 az=-0.99
```

→ đúng, gần `(0, 0, -1)`.

### Front ngẩng lên / +X up

```text
ax=0.99 ay=0.01 az=0.03
```

→ đúng, gần `(+1, 0, 0)` → `gy` âm → pitch âm.

### Front chúi xuống / -X up

```text
ax=-1.01 ay=0.02 az=-0.01
```

→ đúng, gần `(-1, 0, 0)` → `gy` dương → pitch dương.

### Nghiêng sang phải / +Y up

```text
ax=0.00 ay=0.98 az=0.04
```

→ đúng, gần `(0, +1, 0)` 0 → `gx` dương → roll dương.

### Nghiêng sang trái / -Y up

```text
ax=0.02 ay=-1.00 az=0.01
```

→ đúng, gần `(0, -1, 0)` → `gx` âm → roll âm.

### Nhìn từ trên xuống

- Xoay ngược chiều kim đồng hồ / yaw left → `gz` dương → yaw dương.
- Xoay cùng chiều kim đồng hồ / yaw right → `gz` âm → yaw âm.

## 4. Output PID test

### Bước 1: Chỉ test roll

Tạm thời để:

```c
pitch_output = 0;
yaw_output = 0;
```

Mixer chỉ còn:

Tháo cánh, nghiêng drone sang phải.

Đúng phải là:

```text
M2, M3 tăng
M1, M4 giảm
```

Nếu ngược lại, đảo dấu `roll_output` hoặc đảo dấu roll trong mixer.

### Bước 2: Thêm pitch

Khi roll đã đúng, thêm pitch.

Chúc mũi drone xuống.

Đúng phải là:

```text
M1, M2 tăng
M3, M4 giảm
```

Nếu ngược lại, đảo dấu `pitch_output` hoặc đảo dấu pitch trong mixer.

### Bước 3: Thêm yaw

Khi roll và pitch đã đúng, thêm yaw.

Yaw phải, với layout của bạn:

```text
M1 CW
M2 CCW
M3 CW
M4 CCW
```

Đúng phải là:

```text
M2, M4 tăng
M1, M3 giảm
```

Nếu ngược lại, đảo dấu `yaw_output` hoặc đảo dấu yaw trong mixer.

## 5. FFT test

## 6. Nâng cấp

- auto-calibrate gyro bias khi drone thật sự đứng yên | chưa có
- dùng điều kiện landed dựa trên motor + accel norm | chưa có
- calibration accel 6 mặt + đổi về range nhạy hơn khi calibrate | có rồi nhưng chưa chuẩn theo flix
- LPF có init/reset để tránh ramp từ 0 | đã có
- Không dùng accel quá mạnh khi đang bay | đã có
- Failsafe mất sóng | chưa có
