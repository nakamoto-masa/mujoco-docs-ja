ビルド
========

.. WARNING:: OpenUSDサポートは現在実験的なものであり、頻繁に変更される可能性があります。

MuJoCoは事前ビルドされたUSDライブラリに対してビルドする必要があります。これを行うためのユーティリティを提供していますが、独自のUSDライブラリを使用することもできます。

以下の手順では、MuJoCoを ``~/mujoco`` にクローンし、ビルドディレクトリが ``~/mujoco/build`` にあることを前提としています。

.. _usdBuildingUSD:

USDのビルド
------------

事前ビルド済みのUSDライブラリがある場合は、このセクションをスキップできます。

MuJoCoは、USDのビルドプロセスを簡素化するCMakeプロジェクトを提供しています。必要な機能のみを有効にしてUSDをダウンロードおよびビルドします。

.. code-block:: bash

   cd ~/mujoco
   cmake -Bcmake/third_party_deps/openusd/build cmake/third_party_deps/openusd
   cmake --build cmake/third_party_deps/openusd/build

ビルドプロセスをカスタマイズしたい場合は、USDの ``build_usd.py`` スクリプトを使用できます。クローンしたリポジトリディレクトリの外に存在する別のインストールディレクトリを使用することを推奨します。

.. code-block:: bash

   git clone https://github.com/PixarAnimationStudios/OpenUSD
   python OpenUSD/build_scripts/build_usd.py /path/to/my_usd_install_dir

.. _usdEnablingUSD:

USDの有効化
------------

USDがthird_party_deps/openusd CMakeプロジェクトでビルドされた場合、MUJOCO_WITH_USDフラグでUSDサポートを有効にできます。

.. code-block:: bash

   cd ~/mujoco
   cmake -Bbuild -S. -DMUJOCO_WITH_USD=True
   cmake --build build -j 64

それ以外の場合、事前ビルド済みのUSDライブラリがある場合は、pxr_DIRフラグも渡す必要があります。

.. code-block:: bash

   cd ~/mujoco
   cmake -Bbuild -S. -DMUJOCO_WITH_USD=True -Dpxr_DIR=/path/to/my_usd_install_dir
   cmake --build build -j 64


これで :ref:`simulate.cc <saSimulate>` を実行すると、USDファイルをドラッグ＆ドロップできるようになります。

.. code-block:: bash

   simulate
