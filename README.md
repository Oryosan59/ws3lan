# ws3lan - Navigator Control System 🚀

![Build](https://img.shields.io/badge/build-passing-brightgreen)  
リアルタイムセンサ制御・スラスター操作・ゲームパッド入力の統合処理を行う C++ プロジェクトです。BlueRobotics Navigator-lib ライブラリを活用し、ROS などを使用せず軽量で組み込みやすい制御系を構築します。

---

## ✨ 主な機能

- リアルタイムなセンサーデータ（IMU、気圧、水温など）の取得 (※表示・記録は地上局や連携システムに依存)
- ゲームパッドからの入力に基づいたスラスターの精密制御
- ネットワーク経由での遠隔操作指示の受信およびテレメトリデータの送信機能 (※具体的なプロトコルは実装依存)
- モジュール化されたコンポーネント (センサー、ネットワーク、ゲームパッド、スラスター制御)
- 軽量な実装で、Raspberry Piなどのリソースが限られた環境での動作を考慮
- 異常発生時のフェイルセーフ機構 (詳細は後述)

---

## 🛡️ フェイルセーフ機能

本システムには、通信断絶やゲームパッドの接続切れなどの異常事態に備え、以下のフェイルセーフ機能が実装されています。

- **通信断絶時**: 一定時間ゲームパッドや地上局からの入力がない場合、スラスター出力を停止し、安全な状態に移行します。
- **ゲームパッド接続切れ**: ゲームパッドの接続が切れた場合、同様にスラスターを停止します。
- **設定可能なタイムアウト**: フェイルセーフが作動するまでのタイムアウト時間は設定ファイル等で調整可能です。（※ 将来的な拡張または実装詳細を参照）

これにより、予期せぬ状況下でも機体の安全を確保します。

## 🗂️ ディレクトリ構成

```plaintext
WS3/
├── Makefile.mk         # Makefile
├── src/                # ソースファイル (.cpp)
│   ├── main.cpp
│   ├── network.cpp
│   ├── gamepad.cpp
│   ├── thruster_control.cpp
│   └── sensor_data.cpp
├── include/            # ヘッダーファイル (.h/.hpp)
│   ├── network.h
│   ├── gamepad.h
│   ├── thruster_control.h
│   └── sensor_data.h
├── obj/                # コンパイル済オブジェクトファイル (.o)
└── bin/                # 実行ファイル (例: navigator_control)
```

### ✅ この構成のメリット
- **役割の分離**：コードとビルド成果物を明確に分けて管理。
- **スケーラビリティ**：モジュール追加時の見通しが良い。
- **クリーンな管理**：`make clean` で obj/bin ディレクトリのみ削除。

---

## 🛠️ ビルド方法

### 🔧 前提条件
- g++ (C++11以降)
```bash
sudo apt install build-essential
```
- [BlueRobotics Navigator-lib](https://github.com/bluerobotics/navigator-lib) がビルド済みであること（オマケから確認）  
  （デフォルトでは `~/navigator-lib/target/debug` にインストールされている想定）

> **注:**  
> ライブラリのパスは環境変数 `NAVIGATOR_LIB_PATH` で上書き可能です：
>
> ```bash
> make -f Makefile.mk NAVIGATOR_LIB_PATH=/your/custom/path
> ```
>
> 毎回打つのが面倒な場合は、環境変数に書いておくと便利です：
>
> ```bash
> export NAVIGATOR_LIB_PATH=/your/custom/path
> make -f Makefile.mk
> ```


### 🔄 ビルド手順
```bash
git clone https://github.com/Oryosan59/WS3
cd WS3
make -f Makefile.mk
```

### 🧹 クリーンアップ
```bash
make -f Makefile.mk clean
```

---

## 🎯 実行ファイル
ビルド成功後、以下の実行ファイルが生成されます：

```bash
./bin/navigator_control
```

---

## 🎮 基本的な使い方

1.  上記の手順に従ってプロジェクトをビルドし、`./bin/navigator_control` を生成します。
2.  必要に応じて、設定ファイル（存在する場合）を編集し、ネットワーク設定や制御パラメータを調整します。
3.  `./bin/navigator_control` を実行します。
    ```bash
    ./bin/navigator_control
    ```
4.  地上局ソフトウェア（別途準備または開発）を起動し、本システムが動作するデバイスのIPアドレス・ポートに接続します。
5.  対応するゲームパッドをPCまたはRaspberry Piに接続します。
6.  ゲームパッドの入力に応じてスラスターが制御され、センサーデータが地上局に送信されることを確認します。

> **注記:**
> - 具体的なゲームパッドのボタン割り当てや、地上局との通信プロトコルの詳細は、ソースコード内のコメントや関連ドキュメントを参照してください。
> - 初回実行時やハードウェア構成変更後は、キャリブレーションや動作テストを慎重に行ってください。

---

## 🤖 サービスの自動起動 (systemd)

Raspberry Pi 起動時に `navigator_control` を自動的に実行し、万が一プログラムが終了しても自動で再起動するように設定することで、ヘッドレス環境での運用が非常に安定します。ここでは `systemd` を使ったサービス化の方法を説明します。

### 1. サービスファイルの作成

以下のコマンドで、`systemd` のサービス定義ファイルを作成します。

```bash
sudo nano /etc/systemd/system/navigator_control.service
```

### 2. サービスファイルの内容

エディタが開いたら、以下の内容をコピー＆ペーストしてください。`ExecStart` と `WorkingDirectory` のパスは、ご自身の環境に合わせて修正してください。

```ini
[Unit]
Description=Navigator Control Service
After=network.target

[Service]
ExecStart=/home/pi/ws3lan/bin/navigator_control
WorkingDirectory=/home/pi/ws3lan
StandardOutput=journal
StandardError=journal
Restart=always
RestartSec=3
User=pi

[Install]
WantedBy=multi-user.target
```

**🔍 設定のポイント**
- `Restart=always`: プログラムが終了（正常・異常どちらでも）すると、常に再実行されます。
- `RestartSec=3`: 再実行する前に3秒間待機します。これにより、連続クラッシュによるサーバー負荷を避けます。
- `WorkingDirectory`: `config.ini` のような相対パスで指定されたファイルを正しく読み込むために重要です。
- `User=pi`: `pi` ユーザーでプログラムを実行します。ハードウェア（I2C, GPIOなど）へのアクセス権限を持つユーザーを指定してください。
- `StandardOutput=journal`: `printf` や `std::cout` による標準出力を `journald` に記録します。ログの確認に便利です。

### 3. サービスの有効化と起動

ファイルを保存 (`Ctrl+X` -> `Y` -> `Enter`) したら、以下のコマンドでサービスをシステムに登録し、自動起動を有効化します。

```bash
# systemdに新しいサービスファイルを認識させる
sudo systemctl daemon-reload

# OS起動時の自動実行を有効化
sudo systemctl enable navigator_control.service

# 今すぐサービスを起動
sudo systemctl start navigator_control.service
```

### 4. 状態確認とログの表示

サービスが正しく動作しているか確認するには、以下のコマンドを使用します。

```bash
# サービスの実行状態を確認 (Active: active (running) と表示されれば成功)
sudo systemctl status navigator_control.service

# リアルタイムでログを表示 (Ctrl+Cで終了)
journalctl -u navigator_control.service -f
```

これで、Raspberry Pi を再起動しても `navigator_control` が自動で実行されるようになります。

---

## 🔌 外部ライブラリ

- [BlueRobotics Navigator-lib](https://github.com/bluerobotics/navigator-lib)  
  センサ・PWM出力・スラスター制御を行うためのRustベースライブラリ（C/C++バインディングを使用）

---

## 🎁 オマケ：Navigator-lib & Flight Controller のセットアップガイド

このプロジェクトで使用している [BlueRobotics Navigator-lib](https://github.com/bluerobotics/navigator-lib) のバインディング構築方法と、Raspberry Pi 上で Navigator Flight Controller を使うための準備手順を紹介します🛠️✨

---

### 🧱 1. Raspberry Pi Imager のインストール

#### 1.1 Raspberry Pi Imager の入手
- [公式サイト](https://www.raspberrypi.com/software/)から、お使いのOSに対応するバージョンをダウンロード＆インストールしてください。

---

### 💿 2. Raspberry Pi OS イメージの準備

#### 2.1 OSイメージの選定（Lite推奨）
以下のいずれかを選びます。

| アーキテクチャ | ファイル名 | サイズ | ダウンロードリンク |
|----------------|------------|--------|------------------|
| 32-bit (armhf) | 2023-02-21-raspios-bullseye-armhf-lite.img.xz | 約362MB | [DLリンク](https://downloads.raspberrypi.com/raspios_lite_armhf/images/raspios_lite_armhf-2023-02-22/) |
| 64-bit (arm64) | 2023-02-21-raspios-bullseye-arm64-lite.img.xz | 約307MB | [DLリンク](https://downloads.raspberrypi.com/raspios_lite_arm64/images/raspios_lite_arm64-2023-02-22/) |

![Image](https://github.com/user-attachments/assets/a0fb8328-bb0a-4a6b-b84a-f3993df9bd4e)
一番上を選択します

### 2.2 Raspberry Pi Imager で microSD にフラッシュ
1. Imager を起動し、上記でダウンロードしたイメージを選択

    ![Image](https://github.com/user-attachments/assets/c4475b9c-18ef-4990-a089-462760b79c10)
    ![Image](https://github.com/user-attachments/assets/0fea632b-a403-4720-b0ea-051ba5aef1e5)

2. 設定を編集するをクリックして以下を設定：
   - ホスト名、ユーザー名、パスワードの設定
   - Wi-Fi設定（SSIDとパスワード入力。**ステルスSSIDのチェックは外す**）
   - ロケール：
     - タイムゾーン：Asia/Tokyo
     - キーボードレイアウト：jp
   - サービス：SSH を有効化


    ![Image](https://github.com/user-attachments/assets/b457e316-caff-48e7-82dc-f9ec5a33a81a)
    ![Image](https://github.com/user-attachments/assets/92d853b0-1c52-42b7-bb09-f8303e310996)

3. microSD に書き込み開始

---

## 🌐 3. ネットワーク設定
MicroHDMIとキーボードをラズパイに接続し、操作します


### 3.1 固定IP設定（有線LAN）
```bash
sudo nano /etc/dhcpcd.conf
```
ファイル最下部に追記：
```conf
interface eth0
static ip_address=192.168.6.1/24
```
記入が終わったら、終了する: `Ctrl + X`（「Exit」）
「Save modified buffer?」と表示されるので、`Y`（Yes）を押して保存します

---

## 🖥️ 4. Tera Term / ターミナルでの有線SSH接続

### ⚙️ 前提条件
- Raspberry PiとPCをLANケーブルで接続（スイッチングハブ経由または直接接続）
- Raspberry Piに固定IPを設定済み（例: `192.168.6.1`）

---

### 🪟 4.1 Windows：Tera Term で接続する

#### 4.1.1 Tera Term のインストール
1. 公式サイトから Tera Term をダウンロード:  
   [https://ttssh2.osdn.jp/](https://ttssh2.osdn.jp/)
2. インストーラーを実行してインストール

#### 4.1.2 接続手順
1. Tera Term を起動
2. 「ホスト」欄に Raspberry Pi のIPアドレス（例: `192.168.6.1`）を入力
3. 「SSH」を選択し、OKをクリック
4. ユーザー名とパスワードを入力してログイン（例: `pi / raspberry`）

✅ 接続に成功すると、Raspberry Pi のシェル画面が表示されます。

---

### 🍎 4.2 macOS：ターミナルでSSH接続する

#### 4.2.1 標準ターミナルを使用
1. 「ターミナル」アプリを開く（`Command + Space` → "terminal" と入力）
2. 以下のコマンドで接続：

```bash
ssh pi@192.168.1.100
```

3. 最初の接続時に表示される fingerprint の確認メッセージで「yes」と入力
4. パスワードを入力（表示されないが打ててます）

✅ ログインに成功すると、ターミナルに `pi@raspberrypi` のプロンプトが表示されます。

---

### 💡 補足：接続できないときのチェックポイント
- Raspberry Pi の電源が入っているか
- LANケーブルが確実に接続されているか
- Raspberry Pi 側のIPアドレスが正しいか（`ifconfig` または `ip a` で確認可能）
- ファイアウォールの影響がないか

---

## 🌐 5. Windows / macOS での有線LAN固定IP設定ガイド

Raspberry PiとPCをLANケーブルで**直接接続**または**ハブ経由でローカル接続**する場合、お互いのIPアドレスを固定にする必要があります
ここでは、**Raspberry PiのIPを `192.168.6.1` に固定**する前提で、**PC側を `192.168.6.10`** に設定する方法を解説します

---

### 🪟 5.1 Windowsでの設定方法

#### 📌 手順
1. **[設定] → [ネットワークとインターネット] → [アダプターのオプションを変更する]** を開く
2. 有線LAN（例:「イーサネット」）を**右クリック → [プロパティ]**
3. 「インターネットプロトコル バージョン4 (TCP/IPv4)」を選択し**[プロパティ]**
4. 以下のように設定：

| 項目 | 設定内容 |
|------|----------|
| IPアドレス | `192.168.6.10` |
| サブネットマスク | `255.255.255.0` |
| デフォルトゲートウェイ | （空白でOK） |
| DNS | （空白または `8.8.8.8`） |

5. [OK] をクリックして設定を保存

✅ 接続後、Tera Termで `192.168.6.1` にSSH可能になるはずです

---

### 🍎 5.2 macOSでの設定方法

#### 📌 手順
1. **[システム設定] → [ネットワーク]** を開く
2. 左メニューから「有線Ethernet」または「USB LANアダプタ」を選択
3. 「詳細」→ 「TCP/IP」タブを開く
4. 「IPv4の設定」→「手入力（Manually）」を選択
5. 以下を入力：

| 項目 | 設定内容 |
|------|----------|
| IPアドレス | `192.168.6.10` |
| サブネットマスク | `255.255.255.0` |
| ルーター | 空白（または `192.168.1.1`） |

6. [OK] → [適用] をクリック

✅ ターミナルで `ssh pi@192.168.6.1` で接続可能になります

---

### 🧪 接続テスト

PCから以下のように Raspberry Pi に ping を送って確認します：

```bash
ping 192.168.6.1
```

`応答があります` や `bytes from 192.168.6.1` のような表示が出れば、接続成功です！

---


## 🛠️ 6. 必要なツールのインストール
接続後、ターミナルで実行します

```bash
sudo apt update
sudo apt install git i2c-tools
```

---

## 🧭 7. Navigator ハードウェアのセットアップ
 [BlueRobotics Navigator-lib](https://github.com/bluerobotics/navigator-lib)   のバインディング構築方法と、Raspberry Pi 上で Navigator Flight Controller を使うための準備手順を紹介します
 
### 7.1 オーバーレイ設定スクリプトの実行
```bash
sudo su -c 'curl -fsSL https://raw.githubusercontent.com/bluerobotics/blueos-docker/master/install/boards/configure_board.sh | bash'
sudo reboot
```

---

## 🧪 8. Navigator-lib のビルドと実行

### 8.1 依存パッケージのインストール
```bash
sudo apt install cmake git
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

### 8.2 ライブラリの取得とビルド
```bash
git clone https://github.com/bluerobotics/navigator-lib.git
cd navigator-lib/examples/cpp
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug --parallel
```
> **備考**: C++バインディングのビルド結果は `navigator-lib/target/debug` に配置されるため、`Makefile` で `NAVIGATOR_LIB_PATH` にこのパスを指定して利用します。

### 8.3 サンプルの実行
```bash
./build/simple
./build/rainbow
```

以上で、Raspberry Pi 4B + Navigator-lib の準備が完了です！💡

---

## 🧭 作者

**Oryosan59**  
https://github.com/Oryosan59
