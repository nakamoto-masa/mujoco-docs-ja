.. raw:: html

    <div id="fetchlines"/>


.. _API:

====
関数
====

.. tip::
   以下の関数名をクリックすると、GitHub リポジトリのソース実装に移動します。

メインヘッダー `mujoco.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mujoco.h>`_ は多数の関数を公開しています。しかし、ほとんどのユーザーが必要とする関数はごく一部です。

API 関数は以下のように分類できます。

- **メインエントリーポイント**
   - XML ファイルとアセットから :ref:`mjModel` を :ref:`解析とコンパイル<Parseandcompile>` します。
   - :ref:`mj_step` を含む :ref:`メインシミュレーション<Mainsimulation>` のエントリーポイント。

- **サポート関数**
   - :ref:`mjModel` と :ref:`mjData` を必要とする :ref:`サポート<Support>` 関数。
   - :ref:`mj_step` 、 :ref:`mj_forward` 、 :ref:`mj_inverse` から呼び出されるパイプライン :ref:`コンポーネント<Components>` 。
   - シミュレーションパイプラインの :ref:`サブコンポーネント<Subcomponents>` 。
   - :ref:`レイキャスティング<Raycollisions>` 。
   - 各種数量の :ref:`出力<Printing>` 。
   - メモリからアセットを読み込むために使用される :ref:`仮想ファイルシステム<Virtualfilesystem>` 。
   - モデルのコンパイルを高速化するために使用される :ref:`アセットキャッシュ<Assetcache>` 。
   - アセットを読み込むためにリソースプロバイダとインターフェースする :ref:`リソース<Resources>` 。
   - データ構造の :ref:`初期化<Initialization>` 。
   - :ref:`エラーとメモリ<Errorandmemory>` 。
   - :ref:`その他<Miscellaneous>` の関数。

- **可視化、レンダリング、UI**
   - :ref:`抽象的なインタラクション<Interaction>` ：カメラと摂動のマウス制御。
   - :ref:`抽象的な可視化<Visualization-api>` 。
   - :ref:`OpenGL レンダリング<OpenGLrendering>` 。
   - :ref:`UI フレームワーク<UIframework>` 。

- **スレッド、プラグイン、導関数**
   - :ref:`導関数<Derivatives-api>` 。
   - :ref:`スレッド<Thread>` |-| 関連の関数。
   - :ref:`プラグイン<Plugins-api>` |-| 関連の関数。

- **数学**
   - C の :ref:`標準数学<Standardmath>` 関数のエイリアス。
   - :ref:`ベクトル演算<Vectormath>` 。
   - :ref:`疎行列演算<Sparsemath>` 。
   - :ref:`クォータニオン<Quaternions>` 。
   - :ref:`ポーズ変換<Poses>` 。
   - :ref:`行列分解とソルバー<Decompositions>` 。

- **モデル編集**
   - :ref:`アタッチメント<Attachment>` 。
   - :ref:`ツリー要素<AddTreeElements>` 。
   - :ref:`非ツリー要素<AddNonTreeElements>` 。
   - :ref:`アセット<AddAssets>` 。
   - :ref:`検索と取得ユーティリティ<FindAndGetUtilities>` 。
   - :ref:`属性セッター<AttributeSetters>` 。
   - :ref:`属性ゲッター<AttributeGetters>` 。
   - :ref:`Spec ユーティリティ<SpecUtilities>` 。
   - :ref:`要素の初期化<ElementInitialization>` 。
   - :ref:`要素のキャスト<ElementCasting>` 。

.. include:: functions.rst
