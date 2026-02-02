=====
型
=====

MuJoCoは多数の型を定義しています：

- 2つの :ref:`プリミティブ型<tyPrimitive>`
- :ref:`C列挙型<tyEnums>` はカテゴリ値を定義するために使用されます。これらは以下のように分類できます：

  - :ref:`mjModel<tyModelEnums>` で使用される列挙型
  - :ref:`mjData<tyDataEnums>` で使用される列挙型
  - 抽象的な :ref:`可視化<tyVisEnums>` のための列挙型
  - :ref:`OpenGLレンダラー<tyRenderEnums>` で使用される列挙型
  - :ref:`mjUI<tyUIEnums>` ユーザーインターフェースパッケージで使用される列挙型
  - :ref:`エンジンプラグイン<tyPluginEnums>` で使用される列挙型
  - :ref:`プロシージャルモデル操作<tySpecEnums>` で使用される列挙型

  APIはこれらの列挙型を直接使用しないことに注意してください。代わりにintを使用し、ドキュメント/コメントで特定のintが特定の列挙型に対応することを明記しています。これは、APIをコンパイラに依存しないものにしたいためです。C標準では、列挙型の表現に何バイト使用する必要があるかを規定していません。それにもかかわらず、読みやすさを向上させるために、これらを引数として受け取るAPI関数を呼び出す際には、これらの型を使用することをお勧めします。

- :ref:`C構造体型<tyStructure>`。これらは以下のように分類できます：

  - メイン構造体：

    - :ref:`mjModel`
    - :ref:`mjOption` (:ref:`mjModel` に埋め込まれている)
    - :ref:`mjData`

  - :ref:`補助構造体型<tyAuxStructure>` （エンジンでも使用される）
  - :ref:`シミュレーション統計<tyStatStructure>` を収集するための構造体
  - :ref:`抽象可視化<tyVisStructure>` のための構造体
  - :ref:`OpenGLレンダラー<tyRenderStructure>` で使用される構造体
  - :ref:`UIフレームワーク<tyUIStructure>` で使用される構造体
  - :ref:`プロシージャルモデル操作<tySpecStructure>` で使用される構造体
  - :ref:`エンジンプラグイン<tyPluginStructure>` で使用される構造体

- ユーザー定義コールバックのためのいくつかの :ref:`関数型<tyFunction>`
- 詳細な説明を必要とする特定のデータ構造に関する :ref:`tyNotes`



.. _tyPrimitive:

プリミティブ型
---------------

以下の2つの型は `mjtnum.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjtnum.h>`__ で定義されています。


.. _mjtNum:

mjtNum
^^^^^^

これは、シミュレータ全体で使用される浮動小数点型です。デフォルトのビルド構成を使用する場合、 ``mjtNum`` は ``double`` として定義されます。シンボル ``mjUSESINGLE`` が定義されている場合、 ``mjtNum`` は ``float`` として定義されます。

現在、MuJoCoの倍精度版のみが配布されていますが、コードベース全体は単精度でも動作します。将来的に単精度版をリリースする可能性はありますが、倍精度版は常に利用可能です。したがって、倍精度を前提としたユーザーコードを書いても安全です。ただし、単精度または倍精度のどちらでも動作するコードを書くことを推奨します。そのために、常に正しい浮動小数点型で定義される数学ユーティリティ関数を提供しています。

``mjtnum.h`` で ``mjUSESINGLE`` を変更しても、ライブラリがコンパイルされた方法は変わらず、代わりに多数のリンクエラーが発生します。一般的に、プリコンパイルされたMuJoCoと一緒に配布されるヘッダーファイルは、ユーザーが変更すべきではありません。

.. code-block:: C

   // floating point data type and minval
   #ifndef mjUSESINGLE
     typedef double mjtNum;
     #define mjMINVAL    1E-15       // minimum value in any denominator
   #else
     typedef float mjtNum;
     #define mjMINVAL    1E-15f
   #endif


.. _mjtByte:

mjtByte
^^^^^^^

ブール変数を表すために使用されるバイト型です。

.. code-block:: C

   typedef unsigned char mjtByte;


.. _mjtSize:

mjtSize
^^^^^^^

バッファサイズを表すために使用されるサイズ型です。

.. code-block:: C

   typedef int64_t mjtSize;


.. _tyEnums:

列挙型
----------

すべての列挙型は ``mjt`` プレフィックスを使用します。

.. _tyModelEnums:

Model
^^^^^

以下の列挙型は `mjmodel.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`__ で定義されています。


.. _mjtDisableBit:

mjtDisableBit
~~~~~~~~~~~~~

2のべき乗である定数です。これらは :ref:`mjOption` のフィールド ``disableflags`` のビットマスクとして使用されます。実行時、このフィールドは ``m->opt.disableflags`` です。これらの定数の数は ``mjNDISABLE`` で与えられ、これはこれらのフラグのテキスト説明を持つグローバル文字列配列 :ref:`mjDISABLESTRING` の長さでもあります。

.. mujoco-include:: mjtDisableBit


.. _mjtEnableBit:

mjtEnableBit
~~~~~~~~~~~~

2のべき乗である定数です。これらは :ref:`mjOption` のフィールド ``enableflags`` のビットマスクとして使用されます。実行時、このフィールドは ``m->opt.enableflags`` です。これらの定数の数は ``mjNENABLE`` で与えられ、これはこれらのフラグのテキスト説明を持つグローバル文字列配列 :ref:`mjENABLESTRING` の長さでもあります。

.. mujoco-include:: mjtEnableBit


.. _mjtJoint:

mjtJoint
~~~~~~~~

プリミティブジョイント型です。これらの値は ``m->jnt_type`` で使用されます。コメント内の数字は、各ジョイント型が持つ位置座標の数を示しています。ボールジョイントとフリージョイントの回転成分は単位クォータニオンとして表現されることに注意してください。これらは4つの位置座標を持ちますが、それぞれ3つの自由度を持ちます。

.. mujoco-include:: mjtJoint


.. _mjtGeom:

mjtGeom
~~~~~~~

MuJoCoがサポートする幾何学的型です。最初のグループは、モデルで使用できる「公式」ジオム型です。2番目のグループは、モデルでは使用できませんが、ビジュアライザーが装飾要素を追加するために使用するジオム型です。これらの値は ``m->geom_type`` および ``m->site_type`` で使用されます。

.. mujoco-include:: mjtGeom


.. _mjtProjection:

mjtProjection
~~~~~~~~~~~~~

カメラ投影の型です。 ``m->cam_projection`` で使用されます。

.. mujoco-include:: mjtProjection


.. _mjtCamLight:

mjtCamLight
~~~~~~~~~~~

カメラとライトの動的モードで、カメラ/ライトの位置と向きの計算方法を指定します。これらの値は ``m->cam_mode`` および ``m->light_mode`` で使用されます。

.. mujoco-include:: mjtCamLight


.. _mjtLightType:

mjtLightType
~~~~~~~~~~~~

光源の型で、その位置、向き、およびその他のプロパティがシーン内のオブジェクトとどのように相互作用するかを記述します。これらの値は ``m->light_type`` で使用されます。

.. mujoco-include:: mjtLightType


.. _mjtTexture:

mjtTexture
~~~~~~~~~~

テクスチャの型で、テクスチャがどのようにマッピングされるかを指定します。これらの値は ``m->tex_type`` で使用されます。

.. mujoco-include:: mjtTexture


.. _mjtTextureRole:

mjtTextureRole
~~~~~~~~~~~~~~

テクスチャの役割で、レンダラーがテクスチャをどのように解釈すべきかを指定します。MuJoCo組み込みレンダラーはRGBテクスチャのみを使用することに注意してください。これらの値は、マテリアルの配列 ``m->mat_texid`` にテクスチャインデックスを格納するために使用されます。

.. mujoco-include:: mjtTextureRole


.. _mjtColorSpace:

mjtColorSpace
~~~~~~~~~~~~~

テクスチャの色空間エンコーディングの型です。

.. mujoco-include:: mjtColorSpace


.. _mjtIntegrator:

mjtIntegrator
~~~~~~~~~~~~~

数値積分器の型です。これらの値は ``m->opt.integrator`` で使用されます。

.. mujoco-include:: mjtIntegrator

.. _mjtCone:

mjtCone
~~~~~~~

利用可能な摩擦円錐の型です。これらの値は ``m->opt.cone`` で使用されます。

.. mujoco-include:: mjtCone

.. _mjtJacobian:

mjtJacobian
~~~~~~~~~~~

利用可能なヤコビアンの型です。これらの値は ``m->opt.jacobian`` で使用されます。

.. mujoco-include:: mjtJacobian


.. _mjtSolver:

mjtSolver
~~~~~~~~~

利用可能な制約ソルバーアルゴリズムです。これらの値は ``m->opt.solver`` で使用されます。

.. mujoco-include:: mjtSolver


.. _mjtEq:

mjtEq
~~~~~

等式制約の型です。これらの値は ``m->eq_type`` で使用されます。

.. mujoco-include:: mjtEq


.. _mjtWrap:

mjtWrap
~~~~~~~

テンドン巻き付きオブジェクトの型です。これらの値は ``m->wrap_type`` で使用されます。

.. mujoco-include:: mjtWrap


.. _mjtTrn:

mjtTrn
~~~~~~

アクチュエータ伝達の型です。これらの値は ``m->actuator_trntype`` で使用されます。

.. mujoco-include:: mjtTrn


.. _mjtDyn:

mjtDyn
~~~~~~

アクチュエータダイナミクスの型です。これらの値は ``m->actuator_dyntype`` で使用されます。

.. mujoco-include:: mjtDyn


.. _mjtGain:

mjtGain
~~~~~~~

アクチュエータゲインの型です。これらの値は ``m->actuator_gaintype`` で使用されます。

.. mujoco-include:: mjtGain


.. _mjtBias:

mjtBias
~~~~~~~

アクチュエータバイアスの型です。これらの値は ``m->actuator_biastype`` で使用されます。

.. mujoco-include:: mjtBias


.. _mjtObj:

mjtObj
~~~~~~

MuJoCoオブジェクトの型です。これらは、たとえばサポート関数 :ref:`mj_name2id` および :ref:`mj_id2name` でオブジェクト名と整数IDの間の変換に使用されます。

.. mujoco-include:: mjtObj


.. _mjtSensor:

mjtSensor
~~~~~~~~~

センサーの型です。これらの値は ``m->sensor_type`` で使用されます。

.. mujoco-include:: mjtSensor


.. _mjtStage:

mjtStage
~~~~~~~~

これらは :ref:`mj_forwardSkip` および :ref:`mj_inverseSkip` のskipstageパラメータのための計算ステージです。

.. mujoco-include:: mjtStage


.. _mjtDataType:

mjtDataType
~~~~~~~~~~~

これらは可能なセンサーデータ型で、 ``mjData.sensor_datatype`` で使用されます。

.. mujoco-include:: mjtDataType


.. _mjtConDataField:

mjtConDataField
~~~~~~~~~~~~~~~

接触センサーによって返されるデータフィールドの型です。

.. mujoco-include:: mjtConDataField


.. _mjtSameFrame:

mjtSameFrame
~~~~~~~~~~~~

要素とその親ボディのフレーム整列の型です。 :ref:`mj_kinematics` 中に :ref:`mj_local2global` への最後の引数としてショートカットとして使用されます。

.. mujoco-include:: mjtSameFrame


.. _mjtSleepPolicy:

mjtSleepPolicy
~~~~~~~~~~~~~~

ツリーに関連付けられたスリープポリシーです。コンパイラは自動的に ``NEVER`` と ``ALLOWED`` の間で選択しますが、ユーザーはこの選択をオーバーライドできます。 ``INIT`` ポリシー（スリープ状態として初期化）を設定できるのはユーザーのみです。

.. mujoco-include:: mjtSleepPolicy


.. _mjtFlexSelf:

mjtFlexSelf
~~~~~~~~~~~~

フレックス自己衝突midphaseの型です。

.. mujoco-include:: mjtFlexSelf


.. _mjtSDFType:

mjtSDFType
~~~~~~~~~~~

mjc_distanceおよびmjc_gradientを呼び出す際にSDFを組み合わせるために使用される式です。

.. mujoco-include:: mjtSDFType


.. _tyDataEnums:

Data
^^^^

以下の列挙型は `mjdata.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`__ で定義されています。



.. _mjtState:

mjtState
~~~~~~~~

整数ビットフラグとしての状態コンポーネント要素と、これらのフラグのいくつかの便利な組み合わせです。 :ref:`mj_getState`、 :ref:`mj_setState`、 :ref:`mj_stateSize` によって使用されます。

.. mujoco-include:: mjtState


.. _mjtConstraint:

mjtConstraint
~~~~~~~~~~~~~

制約の型です。これらの値はmjModelでは使用されませんが、各シミュレーションタイムステップでアクティブな制約のリストが構築される際に、mjDataフィールド ``d->efc_type`` で使用されます。

.. mujoco-include:: mjtConstraint


.. _mjtConstraintState:

mjtConstraintState
~~~~~~~~~~~~~~~~~~

これらの値は、制約状態を追跡するためにソルバーが内部的に使用します。

.. mujoco-include:: mjtConstraintState


.. _mjtWarning:

mjtWarning
~~~~~~~~~~

警告の型です。警告型の数は ``mjNWARNING`` で与えられ、これは配列 ``mjData.warning`` の長さでもあります。

.. mujoco-include:: mjtWarning


.. _mjtTimer:

mjtTimer
~~~~~~~~

タイマーの型です。タイマー型の数は ``mjNTIMER`` で与えられ、これは配列 ``mjData.timer`` の長さであり、タイマー名を持つ文字列配列 :ref:`mjTIMERSTRING` の長さでもあります。

.. mujoco-include:: mjtTimer


.. _mjtSleepState:

mjtSleepState
~~~~~~~~~~~~~

オブジェクトのスリープ状態です。

.. mujoco-include:: mjtSleepState


.. _tyVisEnums:

Visualization
^^^^^^^^^^^^^

以下の列挙型は `mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`__ で定義されています。


.. _mjtCatBit:

mjtCatBit
~~~~~~~~~

これらは、抽象ビジュアライザーで利用可能なジオムのカテゴリです。ビットマスクは関数 :ref:`mjr_render` でどのカテゴリをレンダリングすべきかを指定するために使用できます。

.. mujoco-include:: mjtCatBit


.. _mjtMouse:

mjtMouse
~~~~~~~~

これらは、抽象ビジュアライザーが認識するマウスアクションです。マウスイベントをインターセプトし、 :ref:`simulate.cc <saSimulate>` で示されているようにこれらのアクションに変換するのはユーザーの責任です。

.. mujoco-include:: mjtMouse


.. _mjtPertBit:

mjtPertBit
~~~~~~~~~~

これらのビットマスクは、マウス摂動の並進成分と回転成分を有効にします。通常のマウスの場合、一度に1つだけ有効にできます。3Dマウス（SpaceNavigator）の場合、両方を同時に有効にできます。これらは ``mjvPerturb.active`` で使用されます。

.. mujoco-include:: mjtPertBit


.. _mjtCamera:

mjtCamera
~~~~~~~~~

これらは可能なカメラの型で、 ``mjvCamera.type`` で使用されます。

.. mujoco-include:: mjtCamera


.. _mjtLabel:

mjtLabel
~~~~~~~~

これらは、テキストラベルを持つことができる抽象可視化要素です。 ``mjvOption.label`` で使用されます。

.. mujoco-include:: mjtLabel


.. _mjtFrame:

mjtFrame
~~~~~~~~

これらは、空間フレームをレンダリングできるMuJoCoオブジェクトです。 ``mjvOption.frame`` で使用されます。

.. mujoco-include:: mjtFrame


.. _mjtVisFlag:

mjtVisFlag
~~~~~~~~~~

これらは配列 ``mjvOption.flags`` のインデックスで、その要素は対応するモデルまたは装飾要素の可視化を有効/無効にします。

.. mujoco-include:: mjtVisFlag


.. _mjtRndFlag:

mjtRndFlag
~~~~~~~~~~

これらは配列 ``mjvScene.flags`` のインデックスで、その要素はOpenGLレンダリング効果を有効/無効にします。

.. mujoco-include:: mjtRndFlag


.. _mjtStereo:

mjtStereo
~~~~~~~~~

これらは可能なステレオレンダリングの型です。 ``mjvScene.stereo`` で使用されます。

.. mujoco-include:: mjtStereo



.. _tyRenderEnums:

Rendering
^^^^^^^^^

以下の列挙型は `mjrender.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`__ で定義されています。


.. _mjtGridPos:

mjtGridPos
~~~~~~~~~~

これらは、テキストオーバーレイの可能なグリッド位置です。関数 :ref:`mjr_overlay` への引数として使用されます。

.. mujoco-include:: mjtGridPos


.. _mjtFramebuffer:

mjtFramebuffer
~~~~~~~~~~~~~~

これらは可能なフレームバッファです。関数 :ref:`mjr_setBuffer` への引数として使用されます。

.. mujoco-include:: mjtFramebuffer


.. _mjtDepthMap:

mjtDepthMap
~~~~~~~~~~~

これらは深度マッピングオプションです。 :ref:`mjrContext` 構造体の ``readPixelDepth`` 属性の値として使用され、 :ref:`mjr_readPixels` が返す深度を ``znear`` から ``zfar`` にどのようにマッピングするかを制御します。

.. mujoco-include:: mjtDepthMap


.. _mjtFontScale:

mjtFontScale
~~~~~~~~~~~~

これらは可能なフォントサイズです。フォントは、3つの異なるサイズで動的ライブラリに保存されている事前定義されたビットマップです。

.. mujoco-include:: mjtFontScale


.. _mjtFont:

mjtFont
~~~~~~~

これらは可能なフォントの型です。

.. mujoco-include:: mjtFont


.. _tyUIEnums:

User Interface
^^^^^^^^^^^^^^

以下の列挙型は `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`__ で定義されています。


.. _mjtButton:

mjtButton
~~~~~~~~~

UIフレームワークで使用されるマウスボタンIDです。

.. mujoco-include:: mjtButton


.. _mjtEvent:

mjtEvent
~~~~~~~~

UIフレームワークで使用されるイベントの型です。

.. mujoco-include:: mjtEvent


.. _mjtItem:

mjtItem
~~~~~~~

UIフレームワークで使用されるアイテムの型です。

.. mujoco-include:: mjtItem


.. _mjtSection:

mjtSection
~~~~~~~~~~

UIセクションの状態です。

.. mujoco-include:: mjtSection



.. _tySpecEnums:

Spec
^^^^

以下の列挙型は `mjspec.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjspec.h>`__ で定義されています。

.. _mjtGeomInertia:

mjtGeomInertia
~~~~~~~~~~~~~~

慣性推論の型です。

.. mujoco-include:: mjtGeomInertia

.. _mjtBuiltin:

mjtBuiltin
~~~~~~~~~~

組み込みプロシージャルテクスチャの型です。

.. mujoco-include:: mjtBuiltin

.. _mjtMark:

mjtMark
~~~~~~~

プロシージャルテクスチャのマークの型です。

.. mujoco-include:: mjtMark

.. _mjtLimited:

mjtLimited
~~~~~~~~~~

制限仕様の型です。

.. mujoco-include:: mjtLimited

.. _mjtAlignFree:

mjtAlignFree
~~~~~~~~~~~~

フリージョイントを慣性フレームに整列させるかどうかです。

.. mujoco-include:: mjtAlignFree

.. _mjtInertiaFromGeom:

mjtInertiaFromGeom
~~~~~~~~~~~~~~~~~~

子ジオムからボディ慣性を推論するかどうかです。

.. mujoco-include:: mjtInertiaFromGeom

.. _mjtOrientation:

mjtOrientation
~~~~~~~~~~~~~~

向き指定子の型です。

.. mujoco-include:: mjtOrientation


.. _tyPluginEnums:

Plugins
^^^^^^^

以下の列挙型は `mjplugin.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjplugin.h>`__ で定義されています。詳細は :ref:`exPlugin` を参照してください。


.. _mjtPluginCapabilityBit:

mjtPluginCapabilityBit
~~~~~~~~~~~~~~~~~~~~~~

エンジンプラグインによって宣言される機能です。

.. mujoco-include:: mjtPluginCapabilityBit



.. _tyStructure:

構造体型
------------

物理シミュレーションのための3つの中心的な構造体型は :ref:`mjModel`、 :ref:`mjOption` (:ref:`mjModel` に埋め込まれている) 、 :ref:`mjData` です。これらの構造に関する入門的な議論は :ref:`Overview<ModelAndData>` にあります。


.. _mjModel:

mjModel
^^^^^^^

これは、MuJoCoモデルを保持するメインデータ構造です。シミュレータによって定数として扱われます。 :ref:`mjModel` のデータ構造に関するいくつかの特定の詳細は、以下の :ref:`tyNotes` にあります。

.. mujoco-include:: mjModel



.. _mjOption:

mjOption
^^^^^^^^

これは、シミュレーションオプションを持つデータ構造です。MJCF要素 :ref:`option <option>` に対応します。そのインスタンスの1つがmjModelに埋め込まれています。

.. mujoco-include:: mjOption


.. _mjData:

mjData
^^^^^^

これは、シミュレーション状態を保持するメインデータ構造です。これは、すべての関数が変更可能な入力を読み取り、出力を書き込むワークスペースです。

.. mujoco-include:: mjData



.. _tyAuxStructure:

Auxiliary
^^^^^^^^^

これらの構造体型はエンジンで使用され、その名前には ``mj`` プレフィックスが付いています。 :ref:`mjVisual` と :ref:`mjStatistic` は :ref:`mjModel` に埋め込まれ、 :ref:`mjContact` は :ref:`mjData` に埋め込まれ、 :ref:`mjVFS` はアセット読み込みに使用されるライブラリレベルの構造体です。


.. _mjVisual:

mjVisual
~~~~~~~~

これは、抽象可視化オプションを持つデータ構造です。MJCF要素 :ref:`visual <visual>` に対応します。そのインスタンスの1つがmjModelに埋め込まれています。

.. mujoco-include:: mjVisual


.. _mjStatistic:

mjStatistic
~~~~~~~~~~~

これは、コンパイラによって事前計算されたモデル統計、またはユーザーによって設定されたモデル統計を持つデータ構造です。MJCF要素 :ref:`statistic <statistic>` に対応します。そのインスタンスの1つがmjModelに埋め込まれています。

.. mujoco-include:: mjStatistic


.. _mjContact:

mjContact
~~~~~~~~~

これは、1つの接触に関する情報を保持するデータ構造です。 ``mjData.contact`` は、衝突検出器によって見つかった接触で実行時に設定されるmjContactデータ構造の事前割り当て配列です。追加の接触情報は、シミュレータによって埋められます。

.. mujoco-include:: mjContact


.. _mjResource:

mjResource
~~~~~~~~~~

リソースは、ファイルシステム内のファイルの抽象化です。nameフィールドはリソースの一意の名前であり、他のフィールドは :ref:`リソースプロバイダー <exProvider>` によって設定されます。

.. mujoco-include:: mjResource


.. _mjVFS:

mjVFS
~~~~~

これは、仮想ファイルシステムのデータ構造です。これはプログラムでのみ構築でき、MJCFには対応するものがありません。

.. mujoco-include:: mjVFS


.. _mjLROpt:

mjLROpt
~~~~~~~

自動 :ref:`アクチュエータ長さ範囲計算<CLengthRange>` を構成するためのオプションです。

.. mujoco-include:: mjLROpt

.. _mjTask:

mjTask
~~~~~~

これは、 :ref:`mjThreadPool` 内で非同期に実行されるタスクの表現です。 :ref:`mjThreadPool` の :ref:`mju_threadPoolEnqueue` メソッドで作成され、完了時にタスクをjoinするために使用されます。

.. mujoco-include:: mjTask

.. _mjThreadPool:

mjThreadPool
~~~~~~~~~~~~

これは、スレッドプールのデータ構造です。これはプログラムでのみ構築でき、MJCFには対応するものがありません。マルチスレッド計算を有効にするには、既存の :ref:`mjThreadPool` へのポインタを ``mjData.threadpool`` に割り当てる必要があります。

.. mujoco-include:: mjThreadPool

.. _tyStatStructure:

Sim statistics
^^^^^^^^^^^^^^

これらの構造体はすべて :ref:`mjData` に埋め込まれ、シミュレーション関連の統計を収集します。


.. _mjWarningStat:

mjWarningStat
~~~~~~~~~~~~~

これは、1つの警告型に関する情報を保持するデータ構造です。 ``mjData.warning`` は、各警告型に対して1つずつのmjWarningStatデータ構造の事前割り当て配列です。

.. mujoco-include:: mjWarningStat


.. _mjTimerStat:

mjTimerStat
~~~~~~~~~~~

これは、1つのタイマーに関する情報を保持するデータ構造です。 ``mjData.timer`` は、各タイマー型に対して1つずつのmjTimerStatデータ構造の事前割り当て配列です。

.. mujoco-include:: mjTimerStat


.. _mjSolverStat:

mjSolverStat
~~~~~~~~~~~~

これは、1つのソルバー反復に関する情報を保持するデータ構造です。 ``mjData.solver`` は、ソルバーの各反復に対して1つずつのmjSolverStatデータ構造の事前割り当て配列で、最大mjNSOLVERまでです。ソルバー反復の実際の数は ``mjData.solver_niter`` によって与えられます。

.. mujoco-include:: mjSolverStat



.. _tyVisStructure:

Visualisation
^^^^^^^^^^^^^

これらの構造体型の名前には ``mjv`` プレフィックスが付いています。

.. _mjvPerturb:

mjvPerturb
~~~~~~~~~~

これは、マウス摂動に関する情報を保持するデータ構造です。

.. mujoco-include:: mjvPerturb


.. _mjvCamera:

mjvCamera
~~~~~~~~~

これは、1つの抽象カメラを記述するデータ構造です。

.. mujoco-include:: mjvCamera


.. _mjvGLCamera:

mjvGLCamera
~~~~~~~~~~~

これは、1つのOpenGLカメラを記述するデータ構造です。

.. mujoco-include:: mjvGLCamera


.. _mjvGeom:

mjvGeom
~~~~~~~

これは、1つの抽象可視化ジオムを記述するデータ構造です。これは、モデルジオムに対応することもあれば、ビジュアライザーによって構築された装飾要素に対応することもあります。

.. mujoco-include:: mjvGeom


.. _mjvLight:

mjvLight
~~~~~~~~

これは、1つのOpenGLライトを記述するデータ構造です。

.. mujoco-include:: mjvLight


.. _mjvOption:

mjvOption
~~~~~~~~~

この構造体には、さまざまな要素の可視化を有効および無効にするオプションが含まれています。

.. mujoco-include:: mjvOption


.. _mjvScene:

mjvScene
~~~~~~~~

この構造体には、OpenGLで3Dシーンをレンダリングするために必要なすべてが含まれています。

.. mujoco-include:: mjvScene


.. _mjvFigure:

mjvFigure
~~~~~~~~~

この構造体には、OpenGLで2Dプロットをレンダリングするために必要なすべてが含まれています。線点などのバッファは事前に割り当てられており、この構造体を引数として関数 :ref:`mjr_figure` を呼び出す前に、ユーザーがそれらを設定する必要があります。

.. mujoco-include:: mjvFigure


.. _tyRenderStructure:

Rendering
^^^^^^^^^

これらの構造体型の名前には ``mjr`` プレフィックスが付いています。

.. _mjrRect:

mjrRect
~~~~~~~

この構造体は長方形を指定します。

.. mujoco-include:: mjrRect


.. _mjrContext:

mjrContext
~~~~~~~~~~

この構造体には、GPUにアップロードされたすべてのOpenGLリソースのIDを含む、カスタムOpenGLレンダリングコンテキストが含まれています。

.. mujoco-include:: mjrContext


.. _tyUIStructure:

User Interface
^^^^^^^^^^^^^^

UIフレームワークの高レベルな説明については、 :ref:`UI` を参照してください。これらの構造体型の名前には ``mjui`` プレフィックスが付いていますが、メインの :ref:`mjUI` 構造体自体は例外です。


.. _mjuiState:

mjuiState
~~~~~~~~~

このC構造体は、ウィンドウ、キーボード、マウスのグローバル状態、入力イベント記述子、およびすべてのウィンドウ矩形（表示されるUI矩形を含む）を表します。アプリケーションごとに1つの ``mjuiState`` しかありません（複数のUIがある場合でも）。この構造体は通常、グローバル変数として定義されます。

.. mujoco-include:: mjuiState


.. _mjuiThemeSpacing:

mjuiThemeSpacing
~~~~~~~~~~~~~~~~

この構造体は、テーマ内のUIアイテムの間隔を定義します。

.. mujoco-include:: mjuiThemeSpacing


.. _mjuiThemeColor:

mjuiThemeColor
~~~~~~~~~~~~~~

この構造体は、テーマ内のUIアイテムの色を定義します。

.. mujoco-include:: mjuiThemeColor


.. _mjuiItem:

mjuiItem
~~~~~~~~

この構造体は、1つのUIアイテムを定義します。

.. mujoco-include:: mjuiItem


.. _mjuiSection:

mjuiSection
~~~~~~~~~~~

この構造体は、UIの1つのセクションを定義します。

.. mujoco-include:: mjuiSection


.. _mjuiDef:

mjuiDef
~~~~~~~

この構造体は、簡素化されたUI構築に使用される定義テーブル内の1つのエントリを定義します。これには、1つのUIアイテムを定義するために必要なすべてが含まれています。ヘルパー関数によっていくつかの変換が実行されるため、複数のmjuiDefを静的テーブルとして定義できます。

.. mujoco-include:: mjuiDef


.. _mjUI:

mjUI
~~~~

このC構造体は、UI全体を表します。同じアプリケーションが、たとえばウィンドウの左側と右側に複数のUIを持つことができます。これは通常、グローバル変数として定義されます。前述のように、これにはサポートされるUIセクション(:ref:`mjuiSection<mjuiSection>`)の最大数とそれぞれのアイテム(:ref:`mjuiItem<mjuiItem>`)の最大数の静的割り当てが含まれています。また、色と間隔のテーマ、有効/無効コールバック、仮想ウィンドウ記述子、テキスト編集状態、マウスフォーカスも含まれています。これらのフィールドの一部は、UIの初期化時に一度だけ設定され、他のフィールドは実行時に変更されます。

.. mujoco-include:: mjUI



.. _tySpecStructure:

Model Editing
^^^^^^^^^^^^^

以下の構造体は `mjspec.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjspec.h>`__ で定義されており、トップレベルの :ref:`mjSpec` 構造体を除いて、 ``mjs`` プレフィックスで始まります。詳細については、 :doc:`Model Editing <../programming/modeledit>` 章を参照してください。

.. _mjSpec:

mjSpec
~~~~~~

モデル仕様です。

.. mujoco-include:: mjSpec


.. _mjsElement:

mjsElement
~~~~~~~~~~

任意の要素に対応する特別な型です。この構造体は、他のすべての要素の最初のメンバーです。低レベルのC++実装では、メンバーとしてではなく、クラス継承を通じて含まれています。継承による包含により、コンパイラは ``mjsElement`` を正しいC++オブジェクトクラスに ``static_cast`` できます。以下の構造体の他のすべての属性（設計上ユーザーが設定可能）とは異なり、 ``mjsElement`` の内容を変更することは許可されておらず、未定義の動作につながります。

.. mujoco-include:: mjsElement


.. _mjsCompiler:

mjsCompiler
~~~~~~~~~~~

コンパイラオプションです。

.. mujoco-include:: mjsCompiler


.. _mjsBody:

mjsBody
~~~~~~~

ボディ仕様です。

.. mujoco-include:: mjsBody


.. _mjsFrame:

mjsFrame
~~~~~~~~

フレーム仕様です。

.. mujoco-include:: mjsFrame


.. _mjsJoint:

mjsJoint
~~~~~~~~

ジョイント仕様です。

.. mujoco-include:: mjsJoint


.. _mjsGeom:

mjsGeom
~~~~~~~

ジオム仕様です。

.. mujoco-include:: mjsGeom


.. _mjsSite:

mjsSite
~~~~~~~

サイト仕様です。

.. mujoco-include:: mjsSite


.. _mjsCamera:

mjsCamera
~~~~~~~~~

カメラ仕様です。

.. mujoco-include:: mjsCamera


.. _mjsLight:

mjsLight
~~~~~~~~

ライト仕様です。

.. mujoco-include:: mjsLight


.. _mjsFlex:

mjsFlex
~~~~~~~

フレックス仕様です。

.. mujoco-include:: mjsFlex


.. _mjsMesh:

mjsMesh
~~~~~~~

メッシュ仕様です。

.. mujoco-include:: mjsMesh


.. _mjsHField:

mjsHField
~~~~~~~~~

ハイトフィールド仕様です。

.. mujoco-include:: mjsHField


.. _mjsSkin:

mjsSkin
~~~~~~~

スキン仕様です。

.. mujoco-include:: mjsSkin


.. _mjsTexture:

mjsTexture
~~~~~~~~~~

テクスチャ仕様です。

.. mujoco-include:: mjsTexture


.. _mjsMaterial:

mjsMaterial
~~~~~~~~~~~

マテリアル仕様です。

.. mujoco-include:: mjsMaterial


.. _mjsPair:

mjsPair
~~~~~~~

ペア仕様です。

.. mujoco-include:: mjsPair


.. _mjsExclude:

mjsExclude
~~~~~~~~~~

除外仕様です。

.. mujoco-include:: mjsExclude


.. _mjsEquality:

mjsEquality
~~~~~~~~~~~

等式仕様です。

.. mujoco-include:: mjsEquality


.. _mjsTendon:

mjsTendon
~~~~~~~~~

テンドン仕様です。

.. mujoco-include:: mjsTendon


.. _mjsWrap:

mjsWrap
~~~~~~~

巻き付きオブジェクト仕様です。

.. mujoco-include:: mjsWrap


.. _mjsActuator:

mjsActuator
~~~~~~~~~~~

アクチュエータ仕様です。

.. mujoco-include:: mjsActuator


.. _mjsSensor:

mjsSensor
~~~~~~~~~

センサー仕様です。

.. mujoco-include:: mjsSensor


.. _mjsNumeric:

mjsNumeric
~~~~~~~~~~

カスタム数値フィールド仕様です。

.. mujoco-include:: mjsNumeric


.. _mjsText:

mjsText
~~~~~~~

カスタムテキスト仕様です。

.. mujoco-include:: mjsText


.. _mjsTuple:

mjsTuple
~~~~~~~~

タプル仕様です。

.. mujoco-include:: mjsTuple


.. _mjsKey:

mjsKey
~~~~~~

キーフレーム仕様です。

.. mujoco-include:: mjsKey


.. _mjsDefault:

mjsDefault
~~~~~~~~~~

デフォルト仕様です。

.. mujoco-include:: mjsDefault


.. _mjsPlugin:

mjsPlugin
~~~~~~~~~

プラグイン仕様です。

.. mujoco-include:: mjsPlugin


.. _mjsOrientation:

mjsOrientation
~~~~~~~~~~~~~~

代替の向き指定子です。

.. mujoco-include:: mjsOrientation


.. _ArrayHandles:

.. _mjByteVec:

.. _mjString:

.. _mjStringVec:

.. _mjIntVec:

.. _mjIntVecVec:

.. _mjFloatVec:

.. _mjFloatVecVec:

.. _mjDoubleVec:

Array handles
~~~~~~~~~~~~~

C++文字列とベクトル型のCハンドルです。Cから使用する場合は、提供されている :ref:`getter<AttributeGetters>` と :ref:`setter<AttributeSetters>` を使用してください。

.. code-block:: C++

   #ifdef __cplusplus
     // C++: defined to be compatible with corresponding std types
     using mjString      = std::string;
     using mjStringVec   = std::vector<std::string>;
     using mjIntVec      = std::vector<int>;
     using mjIntVecVec   = std::vector<std::vector<int>>;
     using mjFloatVec    = std::vector<float>;
     using mjFloatVecVec = std::vector<std::vector<float>>;
     using mjDoubleVec   = std::vector<double>;
     using mjByteVec     = std::vector<std::byte>;
   #else
     // C: opaque types
     typedef void mjString;
     typedef void mjStringVec;
     typedef void mjIntVec;
     typedef void mjIntVecVec;
     typedef void mjFloatVec;
     typedef void mjFloatVecVec;
     typedef void mjDoubleVec;
     typedef void mjByteVec;
   #endif


.. _tyPluginStructure:

Plugins
^^^^^^^

これらの構造体型の名前には ``mjp`` プレフィックスが付いています。詳細は :ref:`exPlugin` を参照してください。


.. _mjpPlugin:

mjpPlugin
~~~~~~~~~

この構造体には、単一のエンジンプラグインの定義が含まれています。これは主に、計算パイプラインのさまざまなフェーズでコンパイラとエンジンによってトリガーされるコールバックのセットを含んでいます。

.. mujoco-include:: mjpPlugin

.. _mjpResourceProvider:

mjpResourceProvider
~~~~~~~~~~~~~~~~~~~

このデータ構造には、 :ref:`リソースプロバイダー <exProvider>` の定義が含まれています。これには、リソースを開いて読み取るために使用されるコールバックのセットが含まれています。

.. mujoco-include:: mjpResourceProvider

.. _tyFunction:

関数型
--------------

MuJoCoコールバックには、対応する関数型があります。これらは `mjdata.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`__ および `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`__ で定義されています。実際のコールバック関数は :doc:`globals<APIglobals>` ページで文書化されています。


.. _tyPhysicsCallbacks:

Physics Callbacks
^^^^^^^^^^^^^^^^^

これらの関数型は :ref:`物理コールバック<glPhysics>` によって使用されます。


.. _mjfGeneric:

mjfGeneric
~~~~~~~~~~

.. code-block:: C

   typedef void (*mjfGeneric)(const mjModel* m, mjData* d);

これは、コールバック :ref:`mjcb_passive` および :ref:`mjcb_control` の関数型です。


.. _mjfConFilt:

mjfConFilt
~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfConFilt)(const mjModel* m, mjData* d, int geom1, int geom2);

これは、コールバック :ref:`mjcb_contactfilter` の関数型です。戻り値は 1: 破棄、0: 衝突チェックを続行です。


.. _mjfSensor:

mjfSensor
~~~~~~~~~

.. code-block:: C

   typedef void (*mjfSensor)(const mjModel* m, mjData* d, int stage);

これは、コールバック :ref:`mjcb_sensor` の関数型です。


.. _mjfTime:

mjfTime
~~~~~~~

.. code-block:: C

   typedef mjtNum (*mjfTime)(void);

これは、コールバック :ref:`mjcb_time` の関数型です。


.. _mjfAct:

mjfAct
~~~~~~

.. code-block:: C

   typedef mjtNum (*mjfAct)(const mjModel* m, const mjData* d, int id);

これは、コールバック :ref:`mjcb_act_dyn`、 :ref:`mjcb_act_gain`、 :ref:`mjcb_act_bias` の関数型です。


.. _mjfCollision:

mjfCollision
~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfCollision)(const mjModel* m, const mjData* d,
                               mjContact* con, int g1, int g2, mjtNum margin);

これは、衝突テーブル :ref:`mjCOLLISIONFUNC` のコールバックの関数型です。


.. _tyUICallbacks:

UI Callbacks
^^^^^^^^^^^^

これらの関数型はUIフレームワークによって使用されます。

.. _mjfItemEnable:

mjfItemEnable
~~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfItemEnable)(int category, void* data);

これは、各アイテムが有効か無効かを判断するためにUIフレームワークが使用する述語関数の関数型です。

.. _tyRPCallbacks:

Resource Provider Callbacks
^^^^^^^^^^^^^^^^^^^^^^^^^^^

これらのコールバックは :ref:`リソースプロバイダー<exProvider>` によって使用されます。

.. _mjfOpenResource:

mjfOpenResource
~~~~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfOpenResource)(mjResource* resource);

このコールバックはリソースを開くためのもので、失敗時にゼロを返します。

.. _mjfReadResource:

mjfReadResource
~~~~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfReadResource)(mjResource* resource, const void** buffer);

このコールバックはリソースを読み取るためのもので、バッファに格納されたバイト数を返し、エラー時に-1を返します。

.. _mjfCloseResource:

mjfCloseResource
~~~~~~~~~~~~~~~~

.. code-block:: C

   typedef void (*mjfCloseResource)(mjResource* resource);

このコールバックはリソースを閉じるためのもので、割り当てられたメモリを解放する責任があります。

.. _mjfGetResourceDir:

mjfGetResourceDir
~~~~~~~~~~~~~~~~~

.. code-block:: C

   typedef void (*mjfGetResourceDir)(mjResource* resource, const char** dir, int* ndir);

このコールバックはリソースのディレクトリを返すためのもので、dirをディレクトリ文字列に設定し、ndirをディレクトリ文字列のサイズとします。

.. _mjfResourceModified:

mjfResourceModified
~~~~~~~~~~~~~~~~~~~

.. code-block:: C

   typedef int (*mjfResourceModified)(const mjResource* resource);

このコールバックは、リソースが最後に読み取られてから変更されたかどうかを確認するためのものです。リソースが最後のオープン以降に変更された場合は正の値を返し、変更されていない場合は0、不明な場合は負の値を返します。


.. _tyNotes:

Notes
-----

このセクションには、MuJoCo構造体型のデータ構造規則に関するその他の注意事項が含まれています。


.. _tyNotesCom:

c-frameの変数
^^^^^^^^^^^^^^^^^

:ref:`mjData` には、内部計算に使用される ``c`` プレフィックスを持つ2つの配列があります：``cdof`` と ``cinert`` で、どちらも :ref:`mj_comPos` によって計算されます。 ``c`` プレフィックスは、量が「c-frame」に関するものであることを意味します。c-frameは、ローカルキネマティックサブツリーの質量中心(``mjData.subtree_com``)にあり、ワールドフレームのように配向されたフレームです。この選択により、グローバル原点から遠く離れたメカニズムの運動学計算の精度が向上します。

``cdof``:
  これらの6D運動ベクトル（3回転、3並進）は、自由度の瞬間軸を記述し、すべてのヤコビアン関数によって使用されます。解析的ヤコビアンに必要な最小限の計算は、 :ref:`mj_kinematics` に続いて :ref:`mj_comPos` です。

``cinert``:
  これらの10ベクトルは、c-frame内のボディの慣性特性を記述し、Composite Rigid Bodyアルゴリズム(:ref:`mj_crb`)によって使用されます。10個の数値は、長さ(6, 3, 1)のパック配列で、意味は次のとおりです：

  ``cinert[0-5]``: ボディの慣性行列の上三角。

  ``cinert[6-8]``: ボディ質量にc-frame原点からのボディCoMのオフセットを掛けたもの。

  ``cinert[9]``: ボディ質量。

.. _tyNotesConvex:

凸包
^^^^^^^^^^^^

凸包記述子は :ref:`mjModel` に格納されています：

.. code-block:: C

   int*      mesh_graphadr;     // graph data address; -1: no graph      (nmesh x 1)
   int*      mesh_graph;        // convex graph data                     (nmeshgraph x 1)

メッシュ ``N`` が :ref:`mjModel` に凸包を格納している場合（これはオプションです）、 ``m->mesh_graphadr[N]`` は ``m->mesh_graph`` 内のメッシュ ``N`` の凸包データのオフセットです。各メッシュの凸包データは、以下の形式のレコードです：

.. code-block:: C

   int numvert;
   int numface;
   int vert_edgeadr[numvert];
   int vert_globalid[numvert];
   int edge_localid[numvert+3*numface];
   int face_globalid[3*numface];

凸包には、完全なメッシュの頂点のサブセットが含まれていることに注意してください。完全なメッシュ内の頂点インデックスを参照するために ``globalid`` という命名法を使用し、凸包内の頂点インデックスを参照するために ``localid`` を使用します。フィールドの意味は次のとおりです：

``numvert``
   凸包内の頂点数。

``numface``
   凸包内の面数。

``vert_edgeadr[numvert]``
   凸包内の各頂点について、これはedge_localid内のその頂点のエッジレコードのオフセットです。

``vert_globalid[numvert]``
   凸包内の各頂点について、これは完全なメッシュ内の対応する頂点インデックスです。

``edge_localid[numvert+3*numface]``
   これには、凸包内の各頂点に対して1つずつ、エッジレコードのシーケンスが含まれています。各エッジレコードは、-1で終了する頂点インデックス（localid形式）の配列です。たとえば、頂点7のレコードが：3、4、5、9、-1 だとします。これは、頂点7が4つのエッジに属し、これらのエッジのもう一方の端は頂点3、4、5、9であることを意味します。このようにして、すべてのエッジは、その2つの頂点のエッジレコードで2回表現されます。閉じた三角メッシュ（ここで使用される凸包など）の場合、エッジ数は ``3*numface/2`` です。したがって、各エッジが2回表現されると、 ``3*numface エッジ`` になります。そして、各エッジレコードの最後（頂点ごとに1つの区切り文字）に区切り文字-1を使用しているため、 ``edge_localid`` の長さは ``numvert+3*numface`` です。

``face_globalid[3*numface]``
   凸包の各面について、これには完全なメッシュ内の3つの頂点のインデックスが含まれています。
