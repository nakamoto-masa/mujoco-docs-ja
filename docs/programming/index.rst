============
プログラミング
==============

.. _inIntro:

はじめに
~~~~~~~~~~~~

本章はMuJoCoプログラミングガイドです。 :doc:`../APIreference/index` のドキュメントは別の章にあります。MuJoCoは、Windows、Linux、macOSに対応した動的ライブラリで、AVX命令をサポートするプロセッサが必要です。このライブラリは、コンパイラに依存しない共有メモリC APIを通じてシミュレータの全機能を公開しています。C++プログラムでも使用できます。

MuJoCoのコードベースは、異なる主要な機能領域に対応するサブディレクトリに整理されています:

Engine
   シミュレータ（または物理エンジン）はCで書かれています。すべてのランタイム計算を担当します。
Parser
   XMLパーサーはC++で書かれています。MJCFモデルとURDFモデルを解析し、mjSpecを介してユーザーに公開される内部的なmjCModel C++オブジェクトに変換できます。
Compiler
   コンパイラはC++で書かれています。パーサーによって構築されたmjCModel C++オブジェクトを受け取り、ランタイムで使用されるmjModel C構造体に変換します。
Abstract visualizer
   抽象ビジュアライザはCで書かれています。シミュレーション状態を表す抽象的な幾何エンティティのリストを生成し、実際のレンダリングに必要なすべての情報を含みます。また、カメラと摂動制御のための抽象的なマウスフックも提供します。
OpenGL renderer
   レンダラーはCで書かれており、固定機能OpenGLに基づいています。最先端のレンダリングエンジンのすべての機能を持っているわけではありません（必要に応じてそのようなエンジンで置き換えることもできます）が、それでも効率的で有益な3Dレンダリングを提供します。
Thread
   スレッディングフレームワーク（MuJoCo 3.0の新機能）はC++で書かれており、Cで公開されています。Taskを非同期に処理するためのThreadPoolインターフェースを提供します。MuJoCoで使用するには、ThreadPoolを作成してmjDataのthread_poolフィールドに割り当てます。
UI framework
   UIフレームワークはCで書かれています。UI要素はOpenGLでレンダリングされます。独自のイベントメカニズムと、キーボードおよびマウス入力用の抽象フックを持っています。コードサンプルではGLFWと一緒に使用していますが、他のウィンドウライブラリでも使用できます。

.. _inStart:

Getting started
~~~~~~~~~~~~~~~

MuJoCoはオープンソースプロジェクトです。ビルド済みの動的ライブラリは、Windows、Linux、macOSで動作するx86_64およびarm64マシン向けに利用可能です。これらは `GitHub Releasesページ <https://github.com/google-deepmind/mujoco/releases>`_ からダウンロードできます。MuJoCoのコアコードを開発または変更する予定のないユーザーは、ビルド済みライブラリの使用を推奨します。これらには定期的にテストしている依存関係の同じバージョンがバンドルされており、パフォーマンスのために調整されたビルドフラグの恩恵を受けられます。ビルド済みライブラリは、標準Cランタイム以外にほぼ完全に自己完結しており、他のライブラリが存在する必要はありません。また、MuJoCoの公開API以外のすべてのシンボルを隠蔽しているため、プロセスにロードされる可能性のある他のライブラリ（MuJoCoが依存するライブラリの他のバージョンを含む）と共存できることが保証されています。

ビルド済みディストリビューションは、Windowsでは単一の.zip、macOSでは.dmg、Linuxでは.tar.gzです。インストーラーはありません。WindowsとLinuxでは、アーカイブを任意のディレクトリに展開するだけです。 ``bin`` サブディレクトリから、ビルド済みのコードサンプルを実行できます。例えば:

.. code-block:: Text

     Windows:           simulate ..\model\humanoid\humanoid.xml
     Linux and macOS:   ./simulate ../model/humanoid/humanoid.xml

ディレクトリ構造は以下の通りです。必要に応じてユーザーが再整理することもでき、動的ライブラリを他のディレクトリにインストールしてパスを適切に設定することもできます。自動的に作成される唯一のファイルは、実行可能ファイルのディレクトリにあるMUJOCO_LOG.TXTで、エラーと警告メッセージが含まれており、いつでも削除できます。

.. code-block:: Text

     bin     - 動的ライブラリ、実行可能ファイル、MUJOCO_LOG.TXT
     doc     - README.txtとREFERENCE.txt
     include - MuJoCoで開発するために必要なヘッダーファイル
     model   - モデルコレクション
     sample  - コードサンプルとビルドに必要なCMakeLists.txt

シミュレータが動作することを確認した後、コードサンプルを再コンパイルして、動作する開発環境があることを確認することもできます。MuJoCoライブラリ自体とは独立してサンプルアプリケーションをビルドするために使用できるクロスプラットフォーム `CMake <https://github.com/google-deepmind/mujoco/blob/main/sample/CMakeLists.txt>`__ セットアップを提供しています。

macOSでは、DMGディスクイメージに ``MuJoCo.app`` が含まれており、ダブルクリックして ``simulate`` GUIを起動できます。 ``MuJoCo.app`` を ``/Application`` にドラッグして、他のアプリと同様にインストールすることもできます。 ``MuJoCo.app`` `Application Bundle <https://developer.apple.com/go/?id=bundle-structure>`__ に加えて、DMGには ``mujoco.framework`` サブディレクトリが含まれており、MuJoCo動的ライブラリとすべての公開ヘッダーが格納されています。Xcodeを使用している場合は、プロジェクトのフレームワーク依存関係としてインポートできます（これはSwiftプロジェクトでも変更なしで動作します）。手動でビルドする場合は、 ``-F`` と ``-framework mujoco`` を使用して、それぞれヘッダー検索パスとライブラリ検索パスを指定できます。

.. _inBuild:

ソースからのビルド
~~~~~~~~~~~~~~~~~~~~

MuJoCoをソースからビルドするには、CMakeと動作するC++17コンパイラがインストールされている必要があります。手順は次のとおりです:

#. ``mujoco`` リポジトリをクローンします: ``git clone https://github.com/deepmind/mujoco.git``
#. 新しいビルドディレクトリを作成し、 ``cd`` で移動します。
#. :shell:`cmake $PATH_TO_CLONED_REPO` を実行してビルドを設定します。
#. ``cmake --build .`` を実行してビルドします。

MuJoCoのビルドシステムは、CMakeの `FetchContent <https://cmake.org/cmake/help/latest/module/FetchContent.html>`_ モジュールを使用して、インターネット経由で上流リポジトリから依存関係を自動的に取得します。

メインのCMakeセットアップは、MuJoCoライブラリ自体とすべてのサンプルアプリケーションをビルドしますが、Pythonバインディングはビルドされません。それらには独自のビルド手順があり、ドキュメントの :doc:`../python` セクションで確認できます。

さらに、CMakeセットアップには、出力ファイルをターゲットディレクトリにコピーして整理するインストールフェーズも実装されています。

5. ディレクトリを選択します: :shell:`cmake $PATH_TO_CLONED_REPO -DCMAKE_INSTALL_PREFIX=<my_install_dir>`
#. ビルド後、 ``cmake --install .`` でインストールします
#. 必要に応じて、Pythonバインディングのビルドに進みます - :ref:`PyBuild` を参照してください。

**注意:**

- Windowsでビルドする場合は、Visual Studio 2019以降を使用し、Windows SDKバージョン10.0.22000以降がインストールされていることを確認してください（詳細は :issue:`862` を参照）。
- ランタイムパフォーマンスを最適化するには、 ``-DCMAKE_BUILD_TYPE=Release`` でビルドしてください

.. tip::
   参考として、MuJoCoの `継続的インテグレーションセットアップ <https://github.com/google-deepmind/mujoco/blob/main/.github/workflows/build.yml>`_ にGitHub上で動作するビルド設定があります。

.. _inBuildDocs:

ドキュメントのビルド
~~~~~~~~~~~~~~~~~~~~

ドキュメントをローカルでビルドしたい場合（例えば、ドキュメントを改善するプルリクエストをテストするため）、次の手順を実行します:

1. ``mujoco`` リポジトリをクローンします: ``git clone https://github.com/deepmind/mujoco.git``
2. ``doc/`` ディレクトリに移動します: ``cd mujoco/doc``
3. 依存関係をインストールします: ``pip install -r requirements.txt``
   |br| MuJoCo Warp APIドキュメントは自動生成され、追加の依存関係が必要です。詳細は `.readthedocs.yml <https://github.com/google-deepmind/mujoco/blob/main/.readthedocs.yml>`__ を参照してください。
4. HTMLをビルドします: ``make html``
5. ブラウザで ``_build/html/index.html`` を開きます。

.. _inHeader:

ヘッダーファイル
~~~~~~~~~~~~~~~~

ディストリビューションには、すべてのプラットフォームで同一のいくつかのヘッダーファイルが含まれています。これらは以下のリンクからも利用可能で、このドキュメントを自己完結させています。

`mujoco.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mujoco.h>`__
   これはメインヘッダーファイルで、MuJoCoを使用するすべてのプログラムに含める必要があります。すべてのAPI関数とグローバル変数を定義し、mjxmacro.h以外のすべての他のヘッダーファイルをインクルードします。
`mjmodel.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`__
   シミュレートされているモデルのランタイム表現である :ref:`mjModel` C構造体を定義します。また、mjModelを定義するために必要なプリミティブ型や他の構造体も定義します。
`mjdata.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`__
   すべての計算がその入力を読み取り、出力を書き込むワークスペースである :ref:`mjData` C構造体を定義します。また、mjDataを定義するために必要なプリミティブ型や他の構造体も定義します。
`mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`__
   抽象ビジュアライザに必要なプリミティブ型と構造体を定義します。
`mjrender.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`__
   OpenGLレンダラーに必要なプリミティブ型と構造体を定義します。
`mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`__
   UIフレームワークに必要なプリミティブ型と構造体を定義します。
`mjtnum.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjtnum.h>`__
   MuJoCoの ``mjtNum`` 浮動小数点型を ``double`` または ``float`` として定義します。 :ref:`mjtNum` を参照してください。
`mjspec.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjspec.h>`__
   :doc:`procedural model editing <modeledit>` に使用される列挙型と構造体を定義します。
`mjplugin.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjplugin.h>`__
   :ref:`エンジンプラグイン<exPlugin>` に必要なデータ構造を定義します。
`mjthread.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjthread.h>`__
   :ref:`スレッド<Thread>` に必要なデータ構造と関数を定義します。
`mjmacro.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmacro.h>`__
   ユーザーコードで役立つCマクロを定義します。
`mjxmacro.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjxmacro.h>`__
   このファイルはオプションであり、mujoco.hには含まれていません。mjModelとmjDataをスクリプト言語にマッピングする自動化や、mjModelとmjDataのすべてのフィールドにアクセスする必要がある他の操作を行える :ref:`Xマクロ <tyXMacro>` を定義します。
`mjexport.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjexport.h>`__
   MuJoCoライブラリから公開シンボルをエクスポートするために使用されるマクロです。このヘッダーはクライアントコードで直接使用すべきではありません。
`mjsan.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjsan.h>`__
   サニタイザー計装でビルドする際に必要な定義です。

.. _inVersion:

バージョンと互換性
~~~~~~~~~~~~~~~~~~~~~~~~~~

MuJoCoは2010年から広く使用されており、非常に成熟しています（バージョン番号付けスキームは非常に保守的ですが）。それにもかかわらず、現在も積極的に開発が続けられており、新機能の多くのエキサイティングなアイデアがあり、ユーザーフィードバックに基づいた変更も行っています。これにより、モデリング言語とAPIの両方に避けられない変更が生じます。最新バージョンへのアップグレードをユーザーに推奨していますが、それが常に実現可能ではないことも認識しています。特に、他の開発者がMuJoCoに依存するソフトウェアをリリースする場合はそうです。そのため、バージョンの競合を回避するための簡単なメカニズムを導入しました。

状況は、既存のコードが特定のバージョンのMuJoCoで開発され、現在は異なるバージョンでコンパイルおよびリンクされている場合により微妙です。そのコードで使用されているAPI関数の定義が変更されている場合、コンパイラまたはリンカーがエラーを生成します。しかし、関数定義が変更されていない場合でも、ソフトウェアバージョンが同じであることをアサートすることが良いアイデアかもしれません。この目的のために、メインヘッダー（mujoco.h）はシンボル :ref:`mjVERSION_HEADER <glNumeric>` を定義し、ライブラリは関数 :ref:`mj_version` を提供します。したがって、ヘッダーとライブラリのバージョンは次のように比較できます:

.. code-block:: C

   // 推奨されるバージョンチェック
   if (mjVERSION_HEADER!=mj_version())
     complain();

メインヘッダーのみがこのシンボルを定義することに注意してください。各ソフトウェアバージョンでリリースされるヘッダーのコレクションは一緒にとどまり、バージョン間で混在しないことを想定しています。浮動小数点比較での複雑さを避けるため、上記のシンボルと関数はバージョン番号の100倍の整数を使用します。たとえば、ソフトウェアバージョン2.1では、シンボルmjVERSION_HEADERは210として定義されます。

.. _inNaming:

命名規則
~~~~~~~~~~~~~~~~~

APIで定義されているすべてのシンボルは、プレフィックス「mj」で始まります。プレフィックスの「mj」の後の文字が、シンボルが属するファミリーを決定します。まず、型定義に対応するプレフィックスをリストします。

``mj``
   コアシミュレーションデータ構造（C構造体）、例えば :ref:`mjModel` 。プレフィックスの後のすべての文字が大文字の場合（例えば :ref:`mjMIN` ）、これはマクロまたはシンボル（#define）です。
``mjt``
   プリミティブ型、例えば :ref:`mjtGeom` 。mjtByteとmjtNumを除き、このファミリーの他のすべての定義は列挙型です。
``mjf``
   コールバック関数型、例えば :ref:`mjfGeneric` 。
``mjv``
   抽象可視化に関連するデータ構造、例えば :ref:`mjvCamera` 。
``mjr``
   OpenGLレンダリングに関連するデータ構造、例えば :ref:`mjrContext` 。
``mjui``
   UIフレームワークに関連するデータ構造、例えば :ref:`mjuiSection` 。
``mjs``
   :doc:`procedural model editing <modeledit>` に関連するデータ構造、例えば :ref:`mjsJoint` 。

次に、関数定義に対応するプレフィックスをリストします。関数プレフィックスは常にアンダースコアで終わることに注意してください。

``mj_``
   コアシミュレーション関数、例えば :ref:`mj_step` 。このような関数のほぼすべては、最初の2つの引数としてmjModelとmjDataへのポインターを持ち、続いて他の引数が続く場合があります。通常、出力はmjDataに書き込まれます。
``mju_``
   ユーティリティ関数、例えば :ref:`mju_mulMatVec` 。これらの関数は、引数としてmjModelとmjDataポインターを持たないという意味で自己完結しています。
``mjv_``
   抽象可視化に関連する関数、例えば :ref:`mjv_updateScene` 。
``mjr_``
   OpenGLレンダリングに関連する関数、例えば :ref:`mjr_render` 。
``mjui_``
   UIフレームワークに関連する関数、例えば :ref:`mjui_update` 。
``mjcb_``
   グローバルコールバック関数ポインター、例えば :ref:`mjcb_control` 。ユーザーは、これらのグローバルポインターをユーザー定義関数に設定することで、カスタムコールバックをインストールできます。
``mjd_``
   導関数を計算する関数、例えば :ref:`mjd_transitionFD` 。
``mjs_``
   :doc:`procedural model editing <modeledit>` の関数、例えば :ref:`mjs_addJoint` 。

.. _inOpenGL:

OpenGLの使用
~~~~~~~~~~~~

MuJoCoのネイティブOpenGLレンダラーの使用については、 :ref:`Rendering` で説明します。レンダリングには、MuJoCoは ``ARB_framebuffer_object`` および ``ARB_vertex_buffer_object`` 拡張機能を備えた互換性プロファイルのOpenGL 1.5を使用します。OpenGLシンボルは、 :ref:`mjr_makeContext` 関数が最初に呼び出されたときに `GLAD <https://github.com/Dav1dde/glad>`_ を介して読み込まれます。これは、MuJoCoライブラリ自体がOpenGLへの明示的な依存関係を持たず、 ``mjr_`` 関数が呼び出されない限り、OpenGLサポートのないシステムでも使用できることを意味します。

MuJoCoの組み込みレンダリング機能を使用するアプリケーションは、適切なOpenGLコンテキスト作成ライブラリに対してリンクし、実行中のスレッドで現在のOpenGLコンテキストがあることを保証する責任があります。WindowsとmacOSでは、オペレーティングシステムによって提供される標準的なOpenGLライブラリがあります。Linuxでは、MuJoCoは現在、X11ウィンドウへのレンダリング用のGLX、ヘッドレスソフトウェアレンダリング用のOSMesa、ハードウェアアクセラレーションヘッドレスレンダリング用のEGLをサポートしています。

バージョン2.1.4以前、MuJoCoはOpenGLシンボルを管理するためにGLADではなくGLEWを使用していました。これには、使用するGL実装に応じて、ビルド時に異なるGLEWライブラリに対してリンクする必要がありました。レンダリングが不要な場合にOpenGL依存関係を管理する必要を避けるため、「nogl」ビルドのライブラリが利用可能でした。GLADへの切り替え以降、OpenGLシンボルは実行時に遅延解決されるため、「nogl」ライブラリは提供されなくなりました。

.. toctree::
    :hidden:

    simulation
    visualization
    ui
    modeledit
    samples
    extension
