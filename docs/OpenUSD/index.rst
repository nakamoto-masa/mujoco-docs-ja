OpenUSD
===========

.. toctree::
    :hidden:

    building
    mjcPhysics
    mjcf_file_format_plugin
    importing
    exporting

.. WARNING:: OpenUSDサポートは現在実験的なものであり、頻繁に変更される可能性があります。

はじめに
------------

本章では、MuJoCoの `OpenUSD <https://openusd.org/release/intro.html>`__ サポートについて説明します。USD（Universal Scene Description）は、Pixarが開発した3Dシーンを記述するためのオープンソースフレームワークです。MuJoCoの統合により、USDの豊富なエコシステムとツール群を活用できます。

OpenUSDとは？
----------------

USDは、3Dデータを記述、合成、シミュレート、共同作業するための高性能で拡張可能なシステムです。元々Pixar Animation Studiosによって開発されたUSDは、現在、ビジュアルエフェクト、アニメーション、ゲーム、ロボティクスなど様々な業界で複雑な3Dワークフローを合理化するために使用されています。異なるソフトウェアアプリケーションが3Dシーン情報を交換するための共通言語を提供します。

なぜOpenUSDが重要なのか？
-----------------------------

USDとMuJoCoの統合には、いくつかの利点があります：

*   **相互運用性:** USDは幅広い3Dコンテンツ作成ツール（例：Houdini、Maya、Blender）でサポートされています。
    これにより、MuJoCoユーザーはこれらのツールで作成されたシーンやアセットを簡単にインポートできます。
*   **豊富なシーン記述:** USDは、ジオメトリ、マテリアル、ライティング、階層構造を含む複雑なシーンを表現するための強力で柔軟な方法を提供します。
*   **コラボレーション:** USDのレイヤリングと合成機能により、強力で効率的な非破壊的オーサリングパイプラインが可能になります。

USDサポート概要
------------------------------------------------------

*   **インポート:** ``.usd``, ``.usda``, ``.usdc``, ``.usdz`` ファイルなどのUSDアセットをMJCF経由で、または :ref:`simulate.cc <saSimulate>` へのドラッグ＆ドロップによってMuJoCoに読み込むことができます。
*   **スキーマ:** MuJoCoは主に、物理特性を表現するための標準的な `UsdPhysics
    <https://openusd.org/dev/api/usd_physics_page_front.html>`__ スキーマを使用します。
*   **拡張:** ``UsdPhysics`` では利用できないMuJoCo固有の機能をカバーするために、カスタムの :doc:`mjcPhysics` スキーマが提供されています。
*   **MJCFファイルフォーマットプラグイン:** :doc:`mjcf_file_format_plugin` により、任意のネイティブUSDアプリケーションでMJCFファイルをUSDレイヤーとして扱うことができます。
*   **エクスポート:** MuJoCoシーンをUSDにエクスポートできます。

USDについてさらに学ぶには？
------------------------------------------

*   `Remedy's Book of USD <https://remedy-entertainment.github.io/USDBook>`__: USDのフレンドリーな入門書。
*   `Official OpenUSD Documentation <https://openusd.org/release/intro.html>`__: APIと実装の詳細に関する公式ドキュメント。
*   `Pixar's USD Introduction <https://graphics.pixar.com/usd/release/index.html>`__: USDの簡単な使用例。
*   `NVIDIA's USD Resources <https://developer.nvidia.com/usd>`__: 主にアセット構造に関するUSDリソース集。
