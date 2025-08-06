# Image Undistort Projesi – Kurulum ve Kullanım Kılavuzu

Bu proje, .bag formatındaki ROS görüntülerini Kalibrasyon verilerine göre düzeltmek (undistort) amacıyla kullanılır. Kamera parametreleri *Kalibr* kalibrasyonu sonucu elde edilen .yaml dosyasından alınır.

---

## Gerekli Bağımlılıklar

### Otomatik Yükleme:
Bu komut ile package.xml dosyasındaki bağımlılıklar kurulur fakat bir hata alınması halinde aşağıdaki komutlarla manuel olarak hata alınan bağımlılıklar tek tek kurulabilir.
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```


###  Manuel Yükleme: 
#### 1. ROS Noetic (Ubuntu 20.04 için)
Kurulu değilse:
```bash
sudo apt install ros-noetic-desktop-full
```

Kurulumdan sonra .bashrc dosyanıza kaynak ekleyin:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 2. Catkin Tools 
Projeyi derlemek ve workspace yönetimi için kullanılır.
```bash
sudo apt install python3-catkin-tools
```

#### 4. Görüntüleme için rqt ve eklentileri
Görüntüleri görselleştirmek için RQT GUI ve rqt_image_view eklentisi gerekir.
```bash
sudo apt install ros-noetic-rqt ros-noetic-rqt-image-view
```

#### 5. USB Kamera Desteği 
Kameradan canlı görüntü almayı sağlar. rostopic: /usb_cam/image_raw bu paketten gelir.
```bash
sudo apt install ros-noetic-usb-cam
```

#### 6. rosbag ile kayıt oynatımı için:
Kayıtlı .bag dosyalarını oynatmak için kullanılır.
```bash
sudo apt install ros-noetic-rosbag
```

#### 7. image_transport 
Görüntü mesajlarını yayınlamak ve abone olmak için kullanılır.
```bash
sudo apt install ros-noetic-image-transport
```

#### 8. cv_bridge 
ROS ile OpenCV arasında görüntü dönüşümü sağlar.
```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-vision-opencv
```

#### 9. rosparam & diğer ROS çekirdek araçları
Parametre yükleme ve yönetim işlemleri için kullanılır (genellikle ROS ile beraber gelir).
Kurulum (gerekirse):
```bash
sudo apt install ros-noetic-rosbash
```

#### 10. Görüntü desteği için:
compressed:JPEG formatında sıkıştırma
```bash
sudo apt install ros-noetic-compressed-image-transport
```
theora: video stream formatı
```bash
sudo apt install ros-noetic-compressed-depth-image-transport
```
compressedDepth: Derinlik görüntüleri için sıkıştırma
```bash
sudo apt install ros-noetic-theora-image-transport
```

---

##  Kurulum ve Çalıştırma Adımları

### 1. Projeyi Klonla
```bash
cd ~/catkin_ws/src
git clone https://github.com/buseSengul/drone_camera_undistort_project_.git
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

### 2. Kalibrasyon Dosyasını Yerleştir
Config klasörüne .yaml dosyanı koy.

Örnek config.yaml:
yaml
```bash
cam0:
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.04674621, -0.05938970, -0.00057779, -0.00004768]
  distortion_model: radtan
  intrinsics: [389.6823, 391.3312, 309.0632, 260.3161]
  resolution: [640, 480]
  rostopic: /usb_cam/image_raw
```

### 3. launch Dosyasını Düzenle
YAML dosyasının adı ve içinde geçen üst key (cam0 gibi) doğru şekilde yazılmalı:

```bash
<launch>
  <arg name="input_camera_name" default="/cam0" />
  <arg name="scale" default="1.0" />
  <arg name="calib_path" default="$(find image_undistort)/config/< yaml dosya adı>.yaml"/>
  <!-- ROS parametresini cam0 altında yükle -->
<rosparam file="$(arg calib_path)" command="load" />

  <node name="image_undistort_node" pkg="image_undistort" type="image_undistort_node" output="screen">
    <param name="input_camera_namespace" value="/<yaml içerisindeki üst key:>" />
    <param name="input_camera_info_from_ros_params" value="true" />
    <param name="output_camera_info_source" value="auto_generated" />
    <param name="scale" value="$(arg scale)" />
    <param name="process_image" value="true" />
    <param name="undistort_image" value="true" />
    <remap from="input/image" to="<görüntünün yayınlandığı ROS topic>" />
  </node>
</launch>
```
      
Eğer canlı kamera görüntüsü alınacaksa:
```bash
<remap from="input/image" to="/camera/image_raw" />
```

Örnek launch:
```bash
<launch>
  <arg name="input_camera_name" default="/cam0" />
  <arg name="scale" default="1.0" />
  <arg name="calib_path" default="$(find image_undistort)/config/arducam_april_tag_640_480-camchain.yaml"/>
  <!-- ROS parametresini cam0 altında yükle -->
<rosparam file="$(arg calib_path)" command="load" />

  <node name="image_undistort_node" pkg="image_undistort" type="image_undistort_node" output="screen">
    <param name="input_camera_namespace" value="/cam0" />
    <param name="input_camera_info_from_ros_params" value="true" />
    <param name="output_camera_info_source" value="auto_generated" />
    <param name="scale" value="$(arg scale)" />
    <param name="process_image" value="true" />
    <param name="undistort_image" value="true" />
    <remap from="input/image" to="/usb_cam/image_raw" />
  </node>
</launch>
```
---

### 4. ROS Master Başlat
```bash
roscore
```

### 5. Başka bir terminalde bag dosyasını oynat:
```bash
rosbag play <dosya_adi.bag> -l
rosbag play arducam_april_tag_640_480.bag -l
```


### 6. Yeni bir terminal aç ve undistort node’unu başlat:
```bash
roslaunch <launch dosyası adı>.launch
roslaunch image_undistort undistort_from_bag.launch
```


### 7. Görüntüyü rqt_image_view veya rqt üzerinden izle:
```bash
rqt
```


---
## Çıktılar ve Gözlemleme

image_undistort_node çalıştırıldığında aşağıdaki ROS topic'leri otomatik olarak oluşur ve yayın yapılır:

### Yayınlanan Topic'ler

| Topic Adı                  | Açıklama                                                                 |
|----------------------------|--------------------------------------------------------------------------|
| /output/image            | Undistort edilmiş görüntü verisi.           |
| /output/image/compressed | Aynı görüntünün sıkıştırılmış hali.           |
| /output/camera_info      | Kamera bilgileri (fx, fy, cx, cy, distortion coefficients, frame_id, vb).|

---

### Topic'leri Gözlemleme Komutları

#### Tüm aktif topic'leri listele
```bash
rostopic list
```
Undistort edilmiş görüntü verisini terminalde görme:
```bash
rostopic echo /output/image
```
Bu komut, görüntünün ham (raw) verisini terminal üzerinde gösterir. Genellikle header, height, width, encoding, data gibi alanları içerir. Görsel değil, veri olarak gözlemlenir.

Kamera kalibrasyon bilgilerini (camera_info) görme:
```bash
rostopic echo /output/camera_info
```
Bu komutla aşağıdaki parametreleri görebilirsiniz:

	•	distortion_model: Kullanılan distortion modeli 
	•	K: Kamera matris parametreleri
	•	D: Distortion  katsayıları
	•	P: Projeksiyon matrisi
	•	frame_id: Görüntü çerçevesi adı
	•	width ve height: Görüntü boyutları

Sıkıştırılmış görüntü verisini görme:
```bash
rostopic echo /output/image/compressed
```

### Kamera parametrelerini .txt olarak kaydetme:
undistort edilmiş kameraya ait yeni kamera parametreleri aşağıdaki komutla .txt dosyası olarak dışa aktarılabilir:

rostopic echo -n1 /output/camera_info > output_camera_info.txt

Bu komut:
	•	/output/camera_info topic’inden 1 adet mesaj alır,
	•	ve onu output_camera_info.txt adlı dosyaya düz metin olarak kaydeder.

🔍 Dosyada yer alan bilgiler:
	•	K: Kamera iç parametre matrisi
	•	D: Distortion katsayıları
	•	P: Projeksiyon matrisi
	•	R: Rotation matrisi
	•	width, height: Görüntü çözünürlüğü
	•	frame_id: Görüntü çerçeve adı

Kaydedilen bu dosya, kalibrasyon doğrulaması veya başka sistemlerde tekrar kullanmak üzere referans olarak saklanabilir.

## Notlar
- .yaml dosyasında K parametresi şart değil, intrinsics varsa yeterlidir.
- cam0 anahtar ismiyle uyumlu şekilde hem yaml hem de launch dosyası yapılandırılmalıdır.

---

