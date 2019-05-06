# 機能
* 対となるRGB/Depth画像を撮影ボタンを押す毎に「capture_tmp」に順次保存  
(RgbDepth,点群pcdデータも追加で保存)

# 撮影条件
* 解像度 : 640✕480
* 15 FPSd

# キャプチャーソフトに使用するセンサの切り替え方
 1. (シリアル番号が分からない場合、アプリを一度起動し確認する。下の場合、PD7110CGC8210717Wがシリアル番号となる)
    ```
    Get device count: 1
    Set Depth Range to Near
    sensor number: 0
    version : DCAM710_c086_pc_sv0.01_R2_20180820_b30
    serial  PD7110CGC8210717W
    acitivate: PD7110CGC8210717W
    Capture Start 
    file count = 0
    fx = 456.636
    fy = 456.226
    cx = 342.983
    cy = 233.82
    ```

 2. cfg/recognition_parameter.tomlのsensor_idを書き換える

# 依存(動作確認済)ライブラリ
* OpenCV : (3.1以上)
* Point Cloud Library : (1.7.2以上)

# PicoZenseSDKのインストール方法
  1. 以下のURLからSDKを入手する(DCAM710, Ubuntu 16.04, Base SDKを選択)
    https://www.picozense.com/en/sdk.html
    
  2. 入手したSDKを解凍する
   
    ```
    tar xvf PicoZenseSDK_Ubuntu16.04_20190316_v2.3.9.2_DCAM710.tar.bz2 
    mkdir -p ~/tmp/
    mv -T PicoZenseSDK_Ubuntu16.04_20190316_v2.3.9.2_DCAM710 ~/tmp/PicoZenseSDK
    ```

  3. install.shを実行する

    ```
    cd ~/tmp/PicoZenseSDK
    sudo ./install.sh
    ```

  4. サンプルコードを実行し、正常にDCAM710が起動するか確認する(DCAM710は起動に20秒ほどかかる)

    ```
    ~/tmp/PicoZenseSDK/Tools/x64/FrameViewer
    ```

  5.  pkg-config用ファイルを作成する(prefixはSDKのインストール先のパスに書き換える)
      以下をコピー＆ペーストし、/usr/lib/pkgconfig/libpicozense.pcとして保存する

    ```
    prefix=/home/inaho-00/tmp/PicoZenseSDK
    exec_prefix=${prefix}
    includedir=${prefix}/Include
    libdir=${exec_prefix}/Lib/x64
    Name: libpicozense
    Description: The Library for Pico Zense
    Version: 1.0.0
    Cflags: -I${includedir}/
    Libs: -L${libdir} -lpicozense_api
    ```

  6. ソフトウェアをビルドし実行する(ビルドが正常に行われるかどうかを確認する)

    ```
    cd ~/tmp
    git clone https://github.com/teaminaho/capture_zense.git
    cd capture_zense
    mkdir -p build
    cd build
    cmake ..
    make -j4
    ./capture_pico_zense
    ```
