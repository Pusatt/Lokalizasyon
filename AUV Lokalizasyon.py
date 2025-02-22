import numpy as np
import matplotlib.pyplot as plt

# Veri girdisi için dosya yolu
file_path = "C:\\Users\\Pusat\\Downloads\\koridor_32_4metre.txt"
TIME_TOLERANCE = 10000  # Zaman toleransı (mikro saniye)

# Verileri okuma ve IMU'lara göre ayırma
def read_and_sort_imu_data(file_path):
    imu1, imu2 = [], []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            imu_id = int(parts[0])
            timestamp = int(parts[1])
            accel_x = int(parts[2])
            accel_y = int(parts[3])
            accel_z = int(parts[4])
            if imu_id == 1:
                imu1.append((timestamp, accel_x, accel_y, accel_z))
            elif imu_id == 2:
                imu2.append((timestamp, accel_x, accel_y, accel_z))
    return sorted(imu1, key=lambda x: x[0]), sorted(imu2, key=lambda x: x[0])

# Zaman damgalarını tolerans dahilinde eşleştirme
def match_data_with_tolerance(imu1, imu2, tolerance):
    matched = []
    i = j = 0
    while i < len(imu1) and j < len(imu2):
        t1, t2 = imu1[i][0], imu2[j][0]
        if abs(t1 - t2) <= tolerance:
            # Yerçekimi düzeltmesi (Z-ekseni için)
            az1 = (imu1[i][3] - 16384) / 16384.0 * 9.81  # 1g çıkarıldı
            az2 = (imu2[j][3] - 16384) / 16384.0 * 9.81
            avg_accel_z = (az1 + az2) / 2

            avg_accel_x = (imu1[i][1] + imu2[j][1]) / 2 / 16384.0 * 9.81
            avg_accel_y = (imu1[i][2] + imu2[j][2]) / 2 / 16384.0 * 9.81

            matched.append((t1, avg_accel_x, avg_accel_y, avg_accel_z))
            i += 1
            j += 1
        elif t1 < t2:
            i += 1
        else:
            j += 1
    return matched

# İvme verilerinden hız ve konum hesaplama (Dead Reckoning)
def dead_reckoning(matched_data):
    if not matched_data:
        return [], []
    
    velocities = []
    positions = []
    prev_time, prev_ax, prev_ay, prev_az = matched_data[0][0], matched_data[0][1], matched_data[0][2], matched_data[0][3]
    vx = vy = vz = 0.0  # Başlangıç hızları
    px = py = pz = 0.0  # Başlangıç konumları
    velocities.append((prev_time, vx, vy, vz))
    positions.append((prev_time, px, py, pz))
    
    for i in range(1, len(matched_data)):
        current_time, ax, ay, az = matched_data[i]
        dt = (current_time - prev_time) / 1e6  # Mikro saniyeden saniyeye çevir
        
        # Ortalama ivme (Trapezoidal integrasyon)
        avg_ax = (prev_ax + ax) / 2
        avg_ay = (prev_ay + ay) / 2
        avg_az = (prev_az + az) / 2
        
        # Hızı güncelle: v = v0 + a * dt
        vx += avg_ax * dt
        vy += avg_ay * dt
        vz += avg_az * dt
        
        # Konumu güncelle: p = p0 + v * dt
        px += vx * dt
        py += vy * dt
        pz += vz * dt
        
        velocities.append((current_time, vx, vy, vz))
        positions.append((current_time, px, py, pz))
        prev_time, prev_ax, prev_ay, prev_az = current_time, ax, ay, az
    
    return velocities, positions

# Grafik çizdirme fonksiyonu
def plot_data(time, accel, velocity, position):
    plt.figure(figsize=(15, 10))
    
    # İvme grafiği
    plt.subplot(3, 1, 1)
    plt.plot(time, accel[:, 0], label='X İvme (m/s²)')
    plt.plot(time, accel[:, 1], label='Y İvme (m/s²)')
    plt.plot(time, accel[:, 2], label='Z İvme (m/s²)')
    plt.title('İvme Verileri')
    plt.xlabel('Zaman (µs)')
    plt.ylabel('İvme (m/s²)')
    plt.legend()
    plt.grid()
    
    # Hız grafiği
    plt.subplot(3, 1, 2)
    plt.plot(time, velocity[:, 0], label='X Hız (m/s)')
    plt.plot(time, velocity[:, 1], label='Y Hız (m/s)')
    plt.plot(time, velocity[:, 2], label='Z Hız (m/s)')
    plt.title('Hız Verileri')
    plt.xlabel('Zaman (µs)')
    plt.ylabel('Hız (m/s)')
    plt.legend()
    plt.grid()
    
    # Konum grafiği
    plt.subplot(3, 1, 3)
    plt.plot(time, position[:, 0], label='X Konum (m)')
    plt.plot(time, position[:, 1], label='Y Konum (m)')
    plt.plot(time, position[:, 2], label='Z Konum (m)')
    plt.title('Konum Verileri')
    plt.xlabel('Zaman (µs)')
    plt.ylabel('Konum (m)')
    plt.legend()
    plt.grid()
    
    plt.tight_layout()
    plt.show()

# Ana işlem
if __name__ == "__main__":
    imu1_sorted, imu2_sorted = read_and_sort_imu_data(file_path)
    matched_data = match_data_with_tolerance(imu1_sorted, imu2_sorted, TIME_TOLERANCE)
    
    # Dead Reckoning ile hız ve konum hesapla
    velocity_data, position_data = dead_reckoning(matched_data)
    
    # Verileri numpy array'e dönüştür
    time = np.array([data[0] for data in matched_data])
    accel = np.array([data[1:] for data in matched_data])
    velocity = np.array([data[1:] for data in velocity_data])
    position = np.array([data[1:] for data in position_data])
    
    # Grafikleri çizdir
    plot_data(time, accel, velocity, position)