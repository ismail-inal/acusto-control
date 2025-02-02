# NASIL KULLANILIR

## Optik Duzenegin Kurulumu

* Kalibrasyonu yapin (genel odak, kamera max roi de olsun)
* Exposure degerini config.json da girin
* lazeri cipin sol alt ve sag ust koselerine denk gelinceki degerlerini config.json da VERTEX adli yere girin.
ilk deger x yani 1. motor sonraki deger y yani 3. motor. vertexin 1. degerleri 2. degerlerinden daha kucuk olmak zorunda.
* kaydetmek istediginiz dosya adini config.json da DIR girin 
* DX ve DY degerleri x ve y de 1 ekran kayacak sekilde ayarlanmali
* AXES motorlarin koordinatlarla iliskisini belirler. PI da kablolarin yeri degismedikce ayni.
* KERNEL_SIZE ise hucreyi iceren goruntunun piksel boyutlaridir.



## Commands

Kodu calistirabilmek icin kutuphaneleri yukleyin

* python -m venv .venv
* source .venv/bin/activate or source .venv/scripts/activate
* pip install -r requirements.txt
