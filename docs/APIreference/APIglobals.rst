=================
グローバル変数
=================

グローバル変数と定数の定義は以下のように分類できます。

- コールバック:

  - :ref:`glError` 。
  - :ref:`glMemory` 。
  - :ref:`glPhysics` 。

- ナローフェーズ衝突関数を含む :ref:`衝突テーブル<glCollision>` 。
- :ref:`文字列定数<glString>` 。
- :ref:`数値定数<glNumeric>` 。
- :ref:`マクロ<Macros>` 。
- :ref:`X マクロ<tyXMacro>` 。

.. _glError:

エラーコールバック
^^^^^^^^^^^^^^^^^^

すべてのユーザーコールバック（つまり、'mjcb' で始まる名前のグローバル関数ポインター）は、最初は NULL に設定されています。これによりコールバックが無効になり、デフォルトの処理が実行されます。コールバックをインストールするには、対応するグローバルポインターを正しい型のユーザー関数に設定するだけです。これらはグローバルであり、モデル固有ではないことに注意してください。したがって、複数のモデルを並列でシミュレーションしている場合、それらは同じコールバックのセットを使用します。


.. _mju_user_error:

mju_user_error
~~~~~~~~~~~~~~

これはメインのエラー関数 :ref:`mju_error` 内から呼び出されます。インストールすると、この関数はデフォルトのエラー処理をオーバーライドします。エラーメッセージを出力した後（またはユーザーが望む他の処理の後）、プログラムを **終了** する必要があります。MuJoCo は mju_error が戻らないという前提で書かれています。戻った場合、ソフトウェアの動作は未定義です。

.. code-block:: C

   extern void (*mju_user_error)(const char*);


.. _mju_user_warning:

mju_user_warning
~~~~~~~~~~~~~~~~

これはメインの警告関数 :ref:`mju_warning` 内から呼び出されます。エラーハンドラーに似ていますが、代わりにプログラムを終了せずに戻る必要があります。

.. code-block:: C

   extern void (*mju_user_warning)(const char*);


.. _glMemory:

メモリコールバック
^^^^^^^^^^^^^^^^^^

メモリコールバックの目的は、ユーザーがカスタムのメモリ割り当ておよび解放メカニズムをインストールできるようにすることです。私たちがこれが有用であることを発見した例の1つは、MuJoCo の MATLAB ラッパーです。ここでは、mex ファイルは永続的なメモリ割り当てのために MATLAB のメモリメカニズムを使用することが期待されています。


.. _mju_user_malloc:

mju_user_malloc
~~~~~~~~~~~~~~~

これがインストールされている場合、MuJoCo ランタイムは必要なすべてのヒープメモリを割り当てるためにこれを使用します（アラインメント付き malloc を使用する代わりに）。ユーザーアロケーターは、8 バイト境界でアラインメントされたメモリを割り当てる必要があります。パーサーとコンパイラは C++ で書かれており、時々 "new" 演算子でメモリを割り当てることがあり、このメカニズムをバイパスすることに注意してください。

.. code-block:: C

   extern void* (*mju_user_malloc)(size_t);


.. _mju_user_free:

mju_user_free
~~~~~~~~~~~~~

これがインストールされている場合、MuJoCo は割り当てたヒープメモリをこの関数を呼び出して解放します（アラインメント付き free を使用する代わりに）。

.. code-block:: C

   extern void (*mju_user_free)(void*);


.. _glPhysics:

物理コールバック
^^^^^^^^^^^^^^^^

物理コールバックは、各種オプションの設定を超えて、シミュレーターの動作を変更するための主要なメカニズムです。オプションはデフォルトパイプラインの動作を制御しますが、コールバックは明確に定義された場所でパイプラインを拡張します。これにより、上級ユーザーは、私たちが考えていない多くの興味深い機能を実装しながら、デフォルトパイプラインを活用できます。他のすべてのコールバックと同様に、自動化されたエラーチェックはありません。代わりに、コールバック関数の作成者が何をしているかを知っていると想定しています。

カスタム物理コールバックには、MJCF では標準的でないパラメータが必要になることがよくあります。これが、MJCF でカスタムフィールドとユーザーデータ配列を提供した主な理由です。アイデアは、必要なユーザーパラメータを入力して MJCF モデルを「計測」し、それらのパラメータを探して対応する計算を実行するコールバックを記述することです。モデルにユーザーパラメータの存在を確認してからアクセスするコールバックを記述することを強く推奨します。そうすることで、通常のモデルが読み込まれたときに、ソフトウェアをクラッシュさせる代わりにコールバックが自動的に無効になります。

.. _mjcb_passive:

mjcb_passive
~~~~~~~~~~~~

これは、関節空間でカスタムパッシブ力を実装するために使用されます。力がデカルト空間でより自然に定義されている場合は、エンドエフェクタのヤコビアンを使用して関節空間にマッピングします。「パッシブ」とは、（物理学のように）正の仕事をしない力を意味するのではなく、単に位置と速度にのみ依存し、制御には依存しない力を意味します。MuJoCo には、バネ、ダンパー、媒体の粘性と密度から生じる標準的なパッシブ力があります。これらは mjcb_passive が呼び出される前に ``mjData.qfrc_passive`` で計算されます。ユーザーコールバックは、このベクトルを上書きするのではなく、追加する必要があります（そうしないと、標準的なパッシブ力が失われます）。

.. code-block:: C

   extern mjfGeneric mjcb_passive;


.. _mjcb_control:

mjcb_control
~~~~~~~~~~~~

これは最も一般的に使用されるコールバックです。制御則を実装し、制御ベクトル ``mjData.ctrl`` に書き込みます。また、 ``mjData.qfrc_applied`` と ``mjData.xfrc_applied`` にも書き込むことができます。これらのベクトルに書き込まれる値は、位置、速度、およびそれらから導出されたすべての量に依存できますが、接触力や制御が指定された後に計算されるその他の量には依存できません。コールバックが後者のフィールドにアクセスする場合、それらの値は現在のタイムステップに対応しません。

制御コールバックは、制御と適用される力が必要とされる直前に、 :ref:`mj_forward` と :ref:`mj_step` 内から呼び出されます。RK 積分器を使用する場合、ステップごとに 4 回呼び出されます。制御と適用される力を指定する別の方法は、 ``mj_step`` の前に設定するか、 ``mj_step1`` と ``mj_step2`` を使用することです。後者のアプローチでは、 ``mj_step1`` によって位置と速度の計算が実行された後に制御を設定できるため、これらの結果を制御の計算に利用できます（mjcb_control を使用するのと同様）。ただし、RK 積分器のサブステップ間で制御を変更する唯一の方法は、制御コールバックを定義することです。

.. code-block:: C

   extern mjfGeneric mjcb_control;

.. _mjcb_contactfilter:

mjcb_contactfilter
~~~~~~~~~~~~~~~~~~

このコールバックは、MuJoCo のデフォルトの衝突フィルタリングを置き換えるために使用できます。インストールされると、この関数は、ブロードフェーズテストに合格した（または MJCF で事前定義されたジオムペアである）各ジオムペアに対して呼び出され、ナローフェーズ衝突の候補となります。デフォルトの処理では、contype と conaffinity のマスク、親子フィルター、および溶接されたボディに関連するその他の考慮事項を使用して、衝突を許可するかどうかを決定します。このコールバックはデフォルトの処理を置き換えますが、メカニズム全体が置き換えられることに注意してください。したがって、たとえば contype/conaffinity を活用したい場合は、コールバック内で再実装する必要があります。

.. code-block:: C

   extern mjfConFilt mjcb_contactfilter;

.. _mjcb_sensor:

mjcb_sensor
~~~~~~~~~~~

このコールバックは、ユーザー定義センサーに対応する ``mjData.sensordata`` のフィールドを設定します。インストールされ、モデルにユーザー定義センサーが含まれている場合に呼び出されます。計算ステージ（mjSTAGE_POS、mjSTAGE_VEL、mjSTAGE_ACC）ごとに 1 回呼び出され、そのステージのすべてのユーザーセンサー値を設定する必要があります。ユーザー定義センサーには、MJCF モデルで定義された次元とデータ型があり、コールバックはこれらを尊重する必要があります。

.. code-block:: C

   extern mjfSensor mjcb_sensor;

.. _mjcb_time:

mjcb_time
~~~~~~~~~

このコールバックをインストールすると、組み込みプロファイラーが有効になり、 ``mjData.timer`` にタイミング統計を保持します。戻り値の型は mjtNum で、時間単位はユーザー次第です。 :ref:`simulate.cc <saSimulate>` は単位が 1 ミリ秒であると想定しています。有用であるためには、コールバックは少なくともマイクロ秒の精度を持つ高解像度タイマーを使用する必要があります。これは、タイミングが測定される計算が非常に高速であるためです。

.. code-block:: C

   extern mjfTime mjcb_time;


.. _mjcb_act_dyn:

mjcb_act_dyn
~~~~~~~~~~~~

このコールバックはカスタム活性化ダイナミクスを実装します。指定されたアクチュエータの ``mjData.act_dot`` の値を返す必要があります。これは活性化状態ベクトル ``mjData.act`` の時間微分です。ユーザーダイナミクス（mjDYN_USER）を持つモデルアクチュエータに対して呼び出されます。そのようなアクチュエータがモデルに存在するがコールバックがインストールされていない場合、それらの時間微分は 0 に設定されます。

.. code-block:: C

   extern mjfAct mjcb_act_dyn;


.. _mjcb_act_gain:

mjcb_act_gain
~~~~~~~~~~~~~

このコールバックはカスタムアクチュエータゲインを実装します。 ``mjModel.actuator_gaintype`` が mjGAIN_USER に設定されている指定されたアクチュエータのゲインを返す必要があります。そのようなアクチュエータがモデルに存在し、このコールバックがインストールされていない場合、それらのゲインは 1 に設定されます。

.. code-block:: C

   extern mjfAct mjcb_act_gain;


.. _mjcb_act_bias:

mjcb_act_bias
~~~~~~~~~~~~~

このコールバックはカスタムアクチュエータバイアスを実装します。 ``mjModel.actuator_biastype`` が mjBIAS_USER に設定されている指定されたアクチュエータのバイアスを返す必要があります。そのようなアクチュエータがモデルに存在し、このコールバックがインストールされていない場合、それらのバイアスは 0 に設定されます。

.. code-block:: C

   extern mjfAct mjcb_act_bias;


.. _glCollision:

衝突テーブル
^^^^^^^^^^^^

.. _mjCOLLISIONFUNC:

mjCOLLISIONFUNC
~~~~~~~~~~~~~~~

ジオムタイプによってインデックス化されたペアワイズ衝突関数のテーブル。右上の三角形のみが使用されます。ユーザーはこれらの関数ポインターをカスタムルーチンに置き換えて、MuJoCo の衝突メカニズムを置き換えることができます。指定されたエントリが NULL の場合、対応するジオムタイプのペアは衝突できません。これらの関数はナローフェーズ衝突にのみ適用されることに注意してください。ブロードフェーズメカニズムは組み込まれており、変更できません。

.. code-block:: C

   extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];


.. _glString:

文字列定数
^^^^^^^^^^

ここで説明する文字列定数は、ユーザーの便宜のために提供されています。これらはオプションのリストの英語名に対応し、GUI のメニューやダイアログに表示できます。コードサンプル :ref:`simulate.cc <saSimulate>` はこれらの使用方法を示しています。


.. _mjDISABLESTRING:

mjDISABLESTRING
~~~~~~~~~~~~~~~

:ref:`mjtDisableBit` で定義された無効化ビットの名前。

.. code-block:: C

   extern const char* mjDISABLESTRING[mjNDISABLE];


.. _mjENABLESTRING:

mjENABLESTRING
~~~~~~~~~~~~~~

:ref:`mjtEnableBit` で定義された有効化ビットの名前。

.. code-block:: C

   extern const char* mjENABLESTRING[mjNENABLE];


.. _mjTIMERSTRING:

mjTIMERSTRING
~~~~~~~~~~~~~

:ref:`mjtTimer` で定義された mjData タイマーの名前。

.. code-block:: C

   extern const char* mjTIMERSTRING[mjNTIMER];


.. _mjLABELSTRING:

mjLABELSTRING
~~~~~~~~~~~~~

:ref:`mjtLabel` で定義されたビジュアルラベリングモードの名前。

.. code-block:: C

   extern const char* mjLABELSTRING[mjNLABEL];


.. _mjFRAMESTRING:

mjFRAMESTRING
~~~~~~~~~~~~~

:ref:`mjtFrame` で定義されたフレーム可視化モードの名前。

.. code-block:: C

   extern const char* mjFRAMESTRING[mjNFRAME];


.. _mjVISSTRING:

mjVISSTRING
~~~~~~~~~~~

:ref:`mjtVisFlag` で定義された抽象可視化フラグの説明。各フラグには 3 つの文字列があり、

以下の意味を持ちます。

[0]: フラグ名;

[1]: :ref:`mjv_defaultOption` で設定されているように、フラグがデフォルトでオンかオフかを示す文字列 "0" または "1";

[2]: :ref:`simulate.cc <saSimulate>` で使用される、推奨されるキーボードショートカットを含む 1 文字の文字列。

.. code-block:: C

   extern const char* mjVISSTRING[mjNVISFLAG][3];


.. _mjRNDSTRING:

mjRNDSTRING
~~~~~~~~~~~

:ref:`mjtRndFlag` で定義された OpenGL レンダリングフラグの説明。各フラグの 3 つの文字列は上記と同じ形式ですが、ここでのデフォルトは :ref:`mjv_makeScene` によって設定されます。

.. code-block:: C

   extern const char* mjRNDSTRING[mjNRNDFLAG][3];



.. _glNumeric:

数値定数
^^^^^^^^

多くの整数定数は、上記のプリミティブ型ですでに文書化されています。さらに、ヘッダーファイルは、ここで文書化されているいくつかの他の定数を定義しています。特に示されていない限り、以下の表の各エントリは `mjmodel.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmodel.h>`_ で定義されています。いくつかの拡張キーコードは `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されていますが、以下の表には示されていません。それらの名前は ``mjKEY_XXX`` の形式です。これらは GLFW キーコードに対応します。

.. list-table::
   :widths: 2 1 8
   :header-rows: 1

   * - シンボル
     - 値
     - 説明
   * - ``mjMINVAL``
     - 1E-15
     - 任意の分母で許可される最小値、および一般的に 0 が許可されない数学演算で許可される最小値。ほとんどすべての場合、MuJoCo はより小さい値を静かに mjMINVAL にクランプします。
   * - ``mjPI``
     - :math:`\pi`
     - :math:`\pi` の値。これは様々な三角関数で使用され、コンパイラでの度からラジアンへの変換にも使用されます。
   * - ``mjMAXVAL``
     - 1E+10
     - mjData.qpos、mjData.qvel、mjData.qacc で許可される最大絶対値。API 関数 :ref:`mj_checkPos` 、 :ref:`mj_checkVel` 、 :ref:`mj_checkAcc` はこの定数を使用して不安定性を検出します。
   * - ``mjMINMU``
     - 1E-5
     - 任意の摩擦係数で許可される最小値。MuJoCo の接触モデルでは、 :at:`condim` 属性で指定されるように、異なる数の摩擦次元を含めることができることを思い出してください。ただし、特定の摩擦次元が含まれている場合、その摩擦はこの定数より小さくすることはできません。より小さい値は自動的にこの定数にクランプされます。
   * - ``mjMINIMP``
     - 0.0001
     - 任意の制約インピーダンスで許可される最小値。より小さい値は自動的にこの定数にクランプされます。
   * - ``mjMAXIMP``
     - 0.9999
     - 任意の制約インピーダンスで許可される最大値。より大きい値は自動的にこの定数にクランプされます。
   * - ``mjMAXCONPAIR``
     - 50
     - ジオムペアごとに生成できる接触点の最大数。MuJoCo の組み込み衝突関数はこの制限を尊重し、ユーザー定義関数もこれを尊重する必要があります。そのような関数は ``mjMAXCONPAIR`` サイズの戻りバッファーで呼び出されます。バッファーにそれ以上の接触を書き込もうとすると、予測不可能な動作を引き起こす可能性があります。
   * - ``mjMAXTREEDEPTH``
     - 50
     - 各ボディとメッシュのバウンディングボリューム階層の最大深度。この大きな制限を超えると、警告が発生し、レイキャスティングができなくなる可能性があります。バランスの取れた階層の場合、これは 1E15 のバウンディングボリュームを意味します。
   * - ``mjMAXFLEXNODES``
     - 27
     - Alessio による何らかの数値で、ドキュメントが必要です。三線形フレックスに関連していると思われますか？
   * - ``mjMINAWAKE``
     - 10
     - ツリーが起こされてから、再びスリープに戻ることが許可されるまでに経過する必要があるタイムステップの最小数。
   * - ``mjNEQDATA``
     - 11
     - 各等式制約を定義するために使用される実数値パラメータの最大数。 ``mjModel.eq_data`` のサイズを決定します。これと次の 5 つの定数は、まだ完全に確定していない配列サイズに対応しています。将来、より精巧な計算に必要な追加パラメータを収容するために、それらを増やす理由があるかもしれません。これが、クォータニオンを表すための配列サイズなど、変更する理由がないものとは対照的に、簡単に変更できるシンボリック定数として維持している理由です。
   * - ``mjNDYN``
     - 10
     - 各アクチュエータの活性化ダイナミクスを定義するために使用される実数値パラメータの最大数。 ``mjModel.actuator_dynprm`` のサイズを決定します。
   * - ``mjNGAIN``
     - 10
     - 各アクチュエータのゲインを定義するために使用される実数値パラメータの最大数。 ``mjModel.actuator_gainprm`` のサイズを決定します。
   * - ``mjNBIAS``
     - 10
     - 各アクチュエータのバイアスを定義するために使用される実数値パラメータの最大数。 ``mjModel.actuator_biasprm`` のサイズを決定します。
   * - ``mjNFLUID``
     - 12
     - 楕円体モデルに必要なジオムごとの流体相互作用パラメータの数。
   * - ``mjNREF``
     - 2
     - 各スカラー制約の参照加速度を定義するために使用される実数値パラメータの最大数。すべての ``mjModel.XXX_solref`` フィールドのサイズを決定します。
   * - ``mjNIMP``
     - 5
     - 各スカラー制約のインピーダンスを定義するために使用される実数値パラメータの最大数。すべての ``mjModel.XXX_solimp`` フィールドのサイズを決定します。
   * - ``mjNSENS``
     - 3
     - センサーパラメータの数。 ``mjModel.sensor_intprm`` のサイズを決定します。
   * - ``mjNSOLVER``
     - 200
     - ``mjData.solver`` にソルバー統計を格納できる反復回数。この配列は、制約ソルバーの各反復に関する診断情報を格納するために使用されます。実際の反復回数は ``mjData.solver_niter`` によって与えられます。
   * - ``mjNISLAND``
     - 20
     - ``mjData.solver`` にソルバー統計を格納できるアイランドの数。この配列は、制約ソルバーの各反復に関する診断情報を格納するために使用されます。ソルバーが実行されたアイランドの実際の数は ``mjData.nsolver_island`` によって与えられます。
   * - ``mjNGROUP``
     - 6
     - :ref:`mjvOption` を介してレンダリングを有効または無効にできるジオム、サイト、ジョイント、テンドン、およびアクチュエータグループの数。 `mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_ で定義されています。
   * - ``mjMAXOVERLAY``
     - 500
     - レンダリング用のオーバーレイテキストの最大文字数。 `mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_ で定義されています。
   * - ``mjMAXLINE``
     - 100
     - 2D 図形（ :ref:`mjvFigure` ）あたりの最大行数。 `mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_ で定義されています。
   * - ``mjMAXLINEPNT``
     - 1001
     - 2D 図形の各行の最大点数。各点には X 座標と Y 座標があるため、バッファー ``mjvFigure.linepnt`` の長さは ``2*mjMAXLINEPNT`` であることに注意してください。 `mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_ で定義されています。
   * - ``mjMAXPLANEGRID``
     - 200
     - 平面をレンダリングするための各次元のグリッド線の最大数。 `mjvisualize.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjvisualize.h>`_ で定義されています。
   * - ``mjNAUX``
     - 10
     - mjrContext で割り当てることができる補助バッファーの数。 `mjrender.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_ で定義されています。
   * - ``mjMAXTEXTURE``
     - 1000
     - 許可される最大テクスチャ数。 `mjrender.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjrender.h>`_ で定義されています。
   * - ``mjMAXTHREAD``
     - 128
     - スレッドプールで使用できる OS スレッドの最大数。 `mjthread.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjthread.h>`_ で定義されています。
   * - ``mjMAXUISECT``
     - 10
     - UI セクションの最大数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjMAXUIITEM``
     - 200
     - UI セクションあたりの最大アイテム数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjMAXUITEXT``
     - 500
     - UI フィールド 'edittext' および 'other' の最大文字数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjMAXUINAME``
     - 40
     - 任意の UI 名の最大文字数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjMAXUIMULTI``
     - 20
     - UI グループのラジオおよび選択アイテムの最大数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjMAXUIEDIT``
     - 5
     - UI 編集リストの最大要素数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjMAXUIRECT``
     - 15
     - UI 矩形の最大数。 `mjui.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjui.h>`_ で定義されています。
   * - ``mjVERSION_HEADER``
     - 3005000
     - MuJoCo ヘッダーのバージョン。これはバージョン文字列 "S.M.P" から式 ``(S * 1e6) + (M * 1e3) + P`` を使用して計算される整数です。たとえば、バージョン 4.2.1 は 4002001 として表されます。mujoco.h で定義されています。API 関数 :ref:`mj_version` は同じ意味の数値を返しますが、コンパイルされたライブラリに対してです。詳細は `VERSIONING.md <https://github.com/google-deepmind/mujoco/blob/main/VERSIONING.md>`__ を参照してください。

.. _Macros:

マクロ
^^^^^^


.. _mjUSESINGLE:

mjUSESINGLE
~~~~~~~~~~~

コンパイル時フラグ、 :ref:`mjtNum` を参照してください。

.. _mjDISABLED:

mjDISABLED
~~~~~~~~~~

.. code-block:: C

   #define mjDISABLED(x) (m->opt.disableflags & (x))

mjModel\* m が定義されていると仮定して、物理オプションを介して特定の標準機能が無効化されているかどうかをチェックします。x は :ref:`mjtDisableBit` 型です。


.. _mjENABLED:

mjENABLED
~~~~~~~~~

.. code-block:: C

   #define mjENABLED(x) (m->opt.enableflags & (x))

mjModel\* m が定義されていると仮定して、物理オプションを介して特定のオプション機能が有効化されているかどうかをチェックします。x は :ref:`mjtEnableBit` 型です。


.. _mjMAX:

mjMAX
~~~~~

.. code-block:: C

   #define mjMAX(a,b) (((a) > (b)) ? (a) : (b))

最大値を返します。mjtNum 型で繰り返し評価を避けるには、関数 :ref:`mju_max` を使用してください。


.. _mjMIN:

mjMIN
~~~~~

.. code-block:: C

   #define mjMIN(a,b) (((a) < (b)) ? (a) : (b))

最小値を返します。mjtNum 型で繰り返し評価を避けるには、関数 :ref:`mju_min` を使用してください。


.. _mjPLUGIN_LIB_INIT:

mjPLUGIN_LIB_INIT
~~~~~~~~~~~~~~~~~

.. code-block:: C

   #define mjPLUGIN_LIB_INIT                                                                 \
     static void _mjplugin_dllmain(void);                                                    \
     mjEXTERNC int __stdcall mjDLLMAIN(void* hinst, unsigned long reason, void* reserved) {  \
       if (reason == 1) {                                                                    \
         _mjplugin_dllmain();                                                                \
       }                                                                                     \
       return 1;                                                                             \
     }                                                                                       \
     static void _mjplugin_dllmain(void)

プラグインをダイナミックライブラリとして登録します。詳細については :ref:`プラグイン登録<exRegistration>` を参照してください。


.. _tyXMacro:

X マクロ
^^^^^^^^

X マクロは、ほとんどのユーザープロジェクトでは必要ありません。これらはモデルを割り当てるために内部的に使用され、このプログラミング技法の使用方法を知っているユーザーも利用できます。実際の定義については、ヘッダーファイル `mjxmacro.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjxmacro.h>`_ を参照してください。これらは、スクリプト言語用の MuJoCo ラッパーを記述する際に特に有用です。ここでは、MuJoCo データ構造に一致する動的構造をプログラムで構築する必要があります。
