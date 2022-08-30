# AFBR-S50 API

[Click here for English README](/README.md)

## まえがき

_AFBR-S50 API_ は[Broadcom Inc](https://www.broadcom.com/)社が販売する，[AFBR-S50 Time-of-Flight Sensor family](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors) に付属するソフトウェア群です．

リポジトリには，AFBR-S50 コアライブラリ, 静的 ANSI-C ライブラリ, およびそれに付随するヘッダファイルとドキュメントが含まれます．

さらに、特定のプロセッサおよび評価ボード用のサンプルおよびデモ プロジェクトが提供します．

## ドキュメント

API リファレンスマニュアルを閲覧するには [こちら](https://broadcom.github.io/AFBR-S50-API/).

## 概要

### ファイル構造

リポジトリは次のように構成されています:

- `/AFBR-S50`: AFBR-S50 API を含みます.
  - `/Lib`: 複数の [Cortex-Mx](https://developer.arm.com/ip-products/processors/cortex-m) アーキテクチャ向けの静的 AFBR-S50 コアライブラリを含みます．
  - `/Include`: AFBR-S50 API が ANSI-C ヘッダファイルとして含まれています．
  - `/Doxygen`: API リファレンス マニュアルを生成するために Doxygen で使用できるドキュメント ファイルが含まれています．
- `/Sources`: 全てのソースコードが含まれます．
  - `/ExampleApp`: UART 接続を介して測定データを配信する簡単なサンプルコード．
  - `/ExplorerApp`: USB または UART を介してシリアル通信を行い，PC 上の専用ソフト AFBR-S50 Explorer GUI に接続するより洗練されたサンプルコード．（ソフトは Broadcom 公式 HP からダウンロード）
  - `/Platform`: 周辺機器ドライバーやハードウェア抽象化レイヤーなどのプラットフォーム固有のコードを含みます．
    - `/NXP_MKLxxZ`: [NXP Kinetis L-Series](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/kl-series-cortex-m0-plus:KINETIS_L_SERIES) に向けたコードが含まれます．(例 [MKL46z](https://www.nxp.com/design/development-boards/freedom-development-boards/mcu-boards/freedom-development-platform-for-kinetis-kl3x-and-kl4x-mcus:FRDM-KL46Z) これは AFBR-S50 Evaluation Kit で使用できます．）
    <!-- - `/STM32F4xx`: The platform code for the [STM32F4 Series](https://www.st.com/en/microcontrollers-microprocessors/stm32f4-series.html) (e.g. [STM32F401RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html) which is used for the new *AFBR-S50 Evaluation Kit*). -->
- `/Projects`: いくつかの IDE のプロジェクト ファイルを含みます．
  - `/MCUXpressoIDE`: [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) で使用できる NXP プロセッサ向けのプロジェクトファイルです．
  <!-- - `/STM32CubeIDE`: Project files for the [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html) for all STM32 processors. -->

### プロジェクト

すべてのプロジェクトは以下にあります `/Projects/<IDE>/<PROJECT>`. 次のプロジェクトが利用可能です:

- `AFBR_S50_ExampleApp_<MCU>`: UART 接続を介して測定データを配信する簡単なサンプルコード．
- `AFBR_S50_ExplorerApp_<MCU>`: USB または UART を介してシリアル通信を行い，PC 上の専用ソフト AFBR-S50 Explorer GUI に接続するより洗練されたサンプルコード．

## Getting Started

### ドキュメントのビルド

ドキュメントは [GitHub Pages](https://broadcom.github.io/AFBR-S50-API/) を使用して既にホストされているため、通常、ドキュメントを作成する必要はありません．

しかしながら、更新されたドキュメントを作成する必要がある場合は、次のツールが必要であり、正しくインストールしてセットアップする必要があります。 セットアップ方法については、ツールのドキュメントを参照してください．

- [Doxygen](https://www.doxygen.nl/)
- [Graphviz](https://graphviz.org/)

セットアップが成功したら、`AFBR-S50` ディレクトリから _doxygen_ を呼び出すことでドキュメントを作成できます:

```bash
$ cd AFBR-S50
$ doxygen
```

現在、`index.html` は `/AFBR-S50/Documentation/html/index.html` にあります．

### MCUXpresso IDE を使用した NXP サンプルのコンパイルおよび実行

[API リファレンスマニュアルの Getting Started](https://broadcom.github.io/AFBR-S50-API/getting_started.html#gs_mcuxpresso) を参照して、[MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)を使用してプロジェクトをセットアップする方法の詳細なガイドを参照してください．

必要な手順を次に示します:

1. このリポジトリをクローンする．
2. [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools-/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)をダウンロードする．
3. NXP MKL46z (もしくは MKL17z) SDK を
   MCUXpresso IDE 内にダウンロードしインポート
4. `/Projects/MCUXpressoIDE`にあるプロジェクトをインポートします．
   - MCUXpresso IDE 上で， _Menu_ > _File_ > _Import..._ > _General_ > _Existing Projects into Workspace_ > _Next_ > Browse and select all required projects > _Finish_ に移動します．
   - "Copy projects into workspace" オプションが無効になっていることを確認してください．
5. プロジェクトのビルド
   - MCUXpresso IDE 上で，_Menu_ > _Project_ > _Build All_
6. プロジェクトのデバッグ
   - MCUXpressoIDE 上の _Project Explorer_ でプロジェクトを選択します．
   - MCUXpresso IDE 上で，_Menu_ > _Run_ > _Debug As_ > _PEMicro probes_ > _OK_
   - デバッガがブレークポイントに到達し次に移動する場合は， _Menu_ > _Run_ > _Resume_

<!-- ### Compile and Run the STM Examples using STM32Cube IDE

Please refer to the [*Getting Started* Section of the *API Reference Manual*](https://broadcom.github.io/AFBR-S50-API/getting_started.html#gs_mcuxpresso) on how to setup the projects using the [STM32Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html). -->

### 別の MCU プラットフォームへの移植

別の MCU プラットフォームで _AFBR-S50 API_ を使用するには、[API リファレンス マニュアルの移植ガイド](https://broadcom.github.io/AFBR-S50-API/porting_guide.html)を参照してください．

Also refer to the special _Porting Guide_ based on a port of the _AFBR-S50 API_ to the [STM32F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) Evaluation Kit with a Cortex-M4 MCU. The guide can be found on the [Broadcom homepage](https://docs.broadcom.com/docs/AFBR-S50-SDK-Porting-Guide-to-Cortex-M4-PG).

Coretex を搭載した，[STM32F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) 評価キットへは AFBR-S50 API の移植に関わる特別なドキュメントが用意されています．参照してください。 このガイドは、[Broadcom ホームページ](https://docs.broadcom.com/docs/AFBR-S50-SDK-Porting-Guide-to-Cortex-M4-PG) にあります。

## サポートを受ける方法

サポートを受けるには，問題が AFBR-S50 API に関連していることを確認してください。
問題が AFBR-S50 ハードウェア に関連している場合は、「support.tof[at]broadcom.com」から対応するサポートにお問い合わせください。

問題が AFBR-S50 API に関連している場合は、[API リファレンス マニュアル](https://broadcom.github.io/AFBR-S50-API/) (特に トラブルシューティング セクション) を読んで理解していることを確認してください。 問題が解決しない場合は、関連する問題がリポジトリの問 issue で見つかるかどうかを確認してください。 それでも問題が解決しない場合は issue を開いてください．

## 貢献する

我々は AFBR-S50 API プロジェクトへの貢献を高く評価します。

貢献のアイデアは次のとおりです。

- ドキュメンテーション作業、
- プラットフォームまたはサンプル コードまたはプロジェクト ファイルの修正/更新、
- プラットフォーム コードの新しいプロセッサに移植する
- サンプル/デモ プロジェクトの追加、
- and more...

あなたの作品が既存の構造とコーディング スタイルにうまく適合することを確認してください．
また、作業を開始する前に，計画された変更を説明する新しい問題を開いて、計画された貢献が受け入れられるかどうかを確認してください．
これは，実装に関する情報を収集するのにも役立ちます。

## Copyright and License

AFBR-S50-API は，BSD 3-Clause License の下で公開されています:

> Copyright (c) 2021, Broadcom Inc
> All rights reserved.
>
> Redistribution and use in source and binary forms, with or without
> modification, are permitted provided that the following conditions are met:
>
> 1. Redistributions of source code must retain the above copyright notice, this
>    list of conditions and the following disclaimer.
>
> 2. Redistributions in binary form must reproduce the above copyright notice,
>    this list of conditions and the following disclaimer in the documentation
>    and/or other materials provided with the distribution.
>
> 3. Neither the name of the copyright holder nor the names of its
>    contributors may be used to endorse or promote products derived from
>    this software without specific prior written permission.
>
> THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
> AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
> IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
> DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
> FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
> DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
> SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
> CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
> OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
> OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
