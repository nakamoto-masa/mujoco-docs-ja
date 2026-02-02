.. _exExtension:

拡張機能
--------

このセクションでは、MuJoCoのユーザー作成の拡張機能のメカニズムについて説明します。現在、拡張性は :ref:`エンジンプラグイン<exPlugin>` と :ref:`リソースプロバイダー<exProvider>` を介して提供されています。

.. _exPlugin:

エンジンプラグイン
~~~~~~~~~~~~~~~~~~

MuJoCo 2.3.0で導入されたエンジンプラグインは、ユーザー定義のロジックをMuJoCoの計算パイプラインのさまざまな部分に挿入することを可能にします。たとえば、カスタムセンサーやアクチュエータタイプをプラグインとして実装できます。プラグイン機能はMJCFモデルのXMLコンテンツで参照されるため、シミュレーション要件がMuJoCoの組み込み機能を超えている場合でも、MJCFはシステムの抽象的な物理的記述のままでいられます。

プラグインメカニズムは、MuJoCoの :ref:`物理コールバック<glPhysics>` の欠点を克服するように設計されました。これらのグローバルコールバック（ :ref:`使用例<siSimulation>` ）は、高速プロトタイピングや、ユーザーがPythonで機能を実装したい場合に依然として利用可能で有用ですが、拡張機能の安定したメカニズムとしては一般的に非推奨です。プラグインメカニズムの中心的な機能は以下の通りです。

- **スレッドセーフティ:** プラグインインスタンス（以下参照）はスレッドローカルであり、衝突を回避します。
- **ステートフル性:** プラグインはステートフルにでき、その状態は正しく（デ）シリアライズされます。
- **相互運用性:** 異なるプラグインが干渉することなく共存できます。

プラグインのユーザーと開発者の両方が、2つの重要な概念を理解する必要があります。

プラグイン
  **プラグイン** は、その機能を実装する関数と静的属性のコレクションであり、 :ref:`mjpPlugin` 構造体にバンドルされています。プラグイン関数は **ステートレス** です。つまり、渡された引数にのみ依存します。プラグインが内部状態を必要とする場合、この状態を宣言し、MuJoCoがそれを管理して渡せるようにします。これにより、完全なシミュレーション状態の（デ）シリアライゼーションが可能になります。したがって、プラグインは機能の「純粋なロジック」部分と見なすことができ、多くの場合Cライブラリとしてバンドルされます。プラグインはモデル要素でもなく、特定のモデル要素に関連付けられているわけでもありません。

プラグインインスタンス
  プラグイン **インスタンス** は、プラグインによって操作される自己完結型のランタイム状態を表します。プラグインロジックが実行されると、インスタンス状態がエンジンによって渡されます。プラグインインスタンス自体は、 :ref:`mjOBJ_PLUGIN<mjtObj>` 型のモデル要素です。 ``[0 nplugin-1]`` のIDを持つ ``mjModel.nplugin`` 個のインスタンスがあります。他の要素と同様に、インスタンスは名前を持つことができ、 :ref:`mj_name2id` と :ref:`mj_id2name` でIDと名前の間をマッピングできます。グローバルテーブルに一度ロードされるプラグインコードとは異なり、同じプラグインの複数のインスタンスを定義でき、他のモデル要素と一対多の関係を持つことができます。

  **一対一:**
    最も単純なケースでは、各インスタンスはモデル内で一度だけ参照されます。たとえば、2つのセンサーが、その値が同じプラグインの2つのプラグインインスタンスによって計算されることを宣言する場合があります。この場合、センサー出力が計算されるたびに、プラグインロジックが個別に実行されます。

  **一対多:**
    あるいは、複数の要素の動作を単一のプラグインインスタンスで支援することができます。これが有用な主なシナリオは2つあります。

  * 異なる要素タイプの値が同じ物理エンティティと計算にリンクされている。たとえば、内部温度計を持つモーターを考えてみてください。これは、アクチュエータとセンサーとして現れ、どちらもトルク出力と温度測定値の両方を計算する同じプラグインインスタンスに関連付けられています。
  * 複数の関連する要素の計算をまとめてバッチ処理することが有利です。たとえば、計算された値がニューラルネットワークの出力である場合などです。ここでの典型的な例は、 ``N`` 個のモーターを装備したロボットで、モーターダイナミクスがニューラルネットワークとしてモデル化されている場合です。この場合、すべてのN個のアクチュエータのトルク出力を、各モーター個別にではなく、単一のフォワードパスで生成する方が大幅に高速になります。

以下では、まずユーザーの観点からプラグインについて説明します。

* プラグイン機能のタイプ。
* プラグインがMJCFモデルでどのように宣言および設定されるか。
* プラグイン状態が :ref:`mjData` にどのように組み込まれるか、およびプラグインインスタンスが存在する場合に :ref:`mjData` 構造体を安全に複製およびシリアライズするためにユーザーが何をする必要があるか。

次に、プラグインのユーザーと開発者の両方に関連するプラグイン登録のロジスティクスについて説明します。その後、プラグイン開発者を対象としたセクションが続きます。

.. _exCapabilities:

プラグイン機能
^^^^^^^^^^^^^^

プラグインは、関連する :ref:`mjpPlugin` 構造体の内容によって記述されます。 ``capabilityflags`` メンバーは、プラグインの機能を記述する整数ビットフィールドであり、ビットのセマンティクスは列挙型 :ref:`mjtPluginCapabilityBit` で定義されています。ビットフィールドを使用することで、プラグインが複数のタイプの計算をサポートできます。現在サポートされているプラグイン機能は以下の通りです。

* アクチュエータプラグイン
* センサープラグイン
* 受動力プラグイン
* 符号付き距離場プラグイン

今後、必要に応じて追加の機能が追加されます。


.. _exDeclaration:

MJCFでの宣言
^^^^^^^^^^^^

まず、 ``<extension><plugin>`` を通じてプラグイン依存関係を宣言する必要があります。モデルが解析されるときに、宣言されているがまだ登録されていないプラグインがある場合（以下を参照）、モデルコンパイルエラーが発生します。単一のMJCF要素のみがプラグインによって支援される場合、インスタンスをその場で暗黙的に作成できます。複数の要素が同じプラグインによって支援される場合、インスタンスの宣言は明示的である必要があります。

.. code:: xml

   <mujoco>
     <extension>
       <plugin plugin="mujoco.test.simple_sensor_plugin"/>
       <plugin plugin="mujoco.test.actuator_sensor_plugin">
         <instance name="explicit_instance"/>
       </plugin>
     </extension>
     ...
     <sensor>
       <plugin name="sensor0" plugin="mujoco.test.simple_sensor_plugin"/>
       <plugin name="sensor1" plugin="mujoco.test.simple_sensor_plugin"/>
       <plugin name="sensor2" instance="explicit_instance"/>
     </sensor>
     ...
     <actuator>
       <plugin name="actuator2" instance="explicit_instance"/>
     </actuator>
   </mujoco>

上記の例では、 ``sensor0`` と ``sensor1`` は、要素間で計算を共有しない単純なプラグインによって支援されているため、プラグイン識別子を直接参照することで各センサーに対してインスタンスが暗黙的に作成されます。対照的に、 ``sensor2`` と ``actuator2`` は計算を共有するプラグインによって支援されているため、明示的に宣言された共有インスタンスを参照する必要があります。


.. _exConfiguration:

MJCFでの設定
^^^^^^^^^^^^

プラグインは、特化した設定可能なパラメータを表すカスタム属性を宣言できます。たとえば、DCモーターモデルは、抵抗、インダクタンス、キャパシタンスを設定属性として公開する場合があります。MJCFでは、これらの属性の値を ``<config>`` 要素を介して指定できます。各 ``<config>`` にはキーと値があります。有効なキーと値はプラグイン開発者によって指定されますが、プラグイン登録時にMuJoCoに宣言されるため、MuJoCoモデルコンパイラが無効な値に対してエラーを発生させることができます。

.. code:: xml

   <mujoco>
     <extension>
       <plugin plugin="mujoco.test.simple_actuator_plugin">
         <instance name="explicit_instance">
           <config key="resistance" value="1.0"/>
           <config key="inductance" value="2.0"/>
         </instance>
       </plugin>
     </extension>
     ...
     <actuator>
       <plugin name="actuator0" instance="explicit_instance"/>
       <plugin name="actuator1" plugin="mujoco.test.simple_actuator_plugin">
           <config key="resistance" value="3.0"/>
           <config key="inductance" value="4.0"/>
       </plugin>
     </actuator>
   </mujoco>

上記の例では、 ``actuator0`` は ``<instance>`` 要素を介して作成および設定された既存のプラグインインスタンスを参照していますが、 ``actuator1`` は新しいプラグインインスタンスをその場で暗黙的に作成および設定しています。 ``actuator0`` に直接 ``<config>`` 子要素を追加するとエラーになることに注意してください。新しいプラグインインスタンスがそこで作成されていないためです。

.. _exPluginState:

プラグイン状態
^^^^^^^^^^^^^^

プラグインコードはステートレスである必要がありますが、個々のプラグインインスタンスは、MuJoCo物理と並行して進化することを意図した時間依存状態を保持することが許可されています。たとえば、熱力学的に結合されたアクチュエータモデルの温度変数などです。別に、プラグインインスタンスが操作の潜在的に高コストな部分をメモ化することも望ましい場合があります。たとえば、事前学習されたニューラルネットワークによって支援されるセンサーまたはアクチュエータプラグインは、モデルコンパイル時に重みをプリロードしたい場合があります。これら2つのタイプのインスタンスごとのプラグインペイロードを区別することが重要です。 **プラグイン状態** という用語は、 *浮動小数点* 値で構成されるプラグインインスタンスの時間依存状態を指し、 **プラグインデータ** という用語は、プラグインの計算の実装詳細と見なされるべきメモ化されたペイロードで構成される *任意のデータ構造* を指します。

重要なことに、プラグインデータは、プラグイン設定属性、プラグイン状態、および :ref:`MuJoCo状態変数<geState>` からのみ再構築可能でなければなりません。これは、プラグインデータがシリアライズ可能であることが期待されず、MuJoCoがデータをコピーまたは保存するときにシリアライズされないことを意味します。一方、プラグイン状態は物理の不可欠な部分と見なされ、物理を忠実に復元するために、MuJoCoの他の状態変数と並んでシリアライズされる必要があります。

プラグインは、 :ref:`mjpPlugin` 構造体の ``nstate`` コールバックを介して、各インスタンスに必要な浮動小数点値の数を宣言する必要があります。この数は、インスタンスの正確な設定に依存する場合があることに注意してください。 :ref:`mj_makeData` 中に、MuJoCoは各プラグインインスタンスに対して :ref:`mjData` の ``plugin_state`` フィールドに必要な数のスロットを割り当てます。 :ref:`mjModel` の ``plugin_stateadr`` フィールドは、各プラグインインスタンスがその状態値を見つけることができる全体の ``plugin_state`` 配列内の位置を示します。

ただし、プラグインデータは、MuJoCoの観点からは完全に不透明です。 :ref:`mj_makeData` 中に、MuJoCoは関連する :ref:`mjpPlugin` から ``init`` コールバックを呼び出します。このコールバックでは、プラグインは機能するために必要な任意のデータ構造を割り当てるか、そうでなければ作成し、そのポインタを作成中の :ref:`mjData` の ``plugin_data`` フィールドに保存できます。 :ref:`mj_deleteData` 中に、MuJoCoは同じ :ref:`mjpPlugin` から ``destroy`` コールバックを呼び出し、プラグインはインスタンスに関連する内部リソースを解放する責任があります。

:ref:`mjData` が :ref:`mj_copyData` を介してコピーされる場合、MuJoCoはプラグイン状態をコピーします。ただし、プラグインコードは、新しくコピーされた :ref:`mjData` のプラグインデータを設定する責任があります。これを容易にするために、MuJoCoは存在する各プラグインインスタンスに対して :ref:`mjpPlugin` から ``copy`` コールバックを呼び出します。

.. _exActuatorAct:

アクチュエータ状態
""""""""""""""""""

ステートフルなアクチュエータプラグインを作成する場合、アクチュエータ状態を保存する場所として2つの選択肢があります。1つは上記のように ``plugin_state`` を使用する方法で、もう1つは :ref:`mjpPlugin` でコールバックを実装することで ``mjData.act`` を使用する方法です。

後者のオプションを使用する場合、アクチュエータプラグインの状態が ``mjData.act`` に追加され、MuJoCoはタイムステップ間で ``mjData.act_dot`` 値を自動的に積分します。このアプローチの1つの利点は、 :ref:`mjd_transitionFD` のような有限差分関数が、ネイティブアクチュエータの場合と同様に機能することです。 ``mjpPlugin.advance`` コールバックは、 ``act_dot`` が積分された後に呼び出され、組み込みの積分器が適切でない場合、アクチュエータプラグインはその時点で ``act`` 値を上書きできます。

ユーザーは、アクチュエータプラグインに :ref:`dyntype<actuator-plugin-dyntype>` 属性を指定して、ユーザー入力とアクチュエータ状態の間にフィルタまたは積分器を導入できます。その場合、 ``dyntype`` によって導入される状態変数は、 ``act`` 配列内のプラグインの状態変数の *後* に配置されます。

.. _exRegistration:

登録
^^^^

プラグインは、MJCFモデルで参照される前にMuJoCoに登録する必要があります。

特定のアプリケーションをサポートすることを意図したワンオフプラグイン（またはモデルの問題のトラブルシューティングに役立つように実装された使い捨てプラグイン）は、アプリケーションに静的にリンクできます。これは、 ``main`` 関数で :ref:`mjpPlugin` 構造体を準備し、それを :ref:`mjp_registerPlugin` に渡してMuJoCoに登録するのと同じくらい簡単です。

一般的に、再利用可能なプラグインは動的ライブラリとしてパッケージ化されることが期待されます。1つ以上のMuJoCoプラグインを含む動的ライブラリは、ライブラリがロードされたときにすべてのプラグインが登録されていることを確認する必要があります。GCC互換コンパイラでは、 ``__attribute__((constructor))`` で宣言された関数内で :ref:`mjp_registerPlugin` を呼び出すことでこれを実現できますが、MSVCではDLLエントリポイント（通常は ``DllMain`` として知られています）でこれを行うことができます。MuJoCoは、使用されるコンパイラに応じてこれらの構造のいずれかに展開される便利なマクロ :ref:`mjPLUGIN_LIB_INIT` を提供します。

上記のように動的ライブラリとして配信されるプラグインのユーザーは、関数 :ref:`mj_loadPluginLibrary` を使用してライブラリをロードできます。これは、MuJoCoプラグインを含む動的ライブラリをロードする推奨方法です（たとえば、 ``dlopen`` や ``LoadLibraryA`` を直接呼び出すのではなく）。MuJoCoが動的ライブラリにプラグインを自動登録させることを期待する正確な方法は将来変更される可能性がありますが、 :ref:`mj_loadPluginLibrary` もベストプラクティスを反映するように進化することが期待されます。

任意のユーザー提供のMJCFモデルをロードできる必要があるアプリケーションの場合、特定のディレクトリ内で見つかったすべての動的ライブラリを自動的にスキャンしてロードすることが望ましい場合があります。プラグインを必要とするMJCFを持参するユーザーは、必要なプラグインライブラリを関連するディレクトリに配置するように指示できます。たとえば、これは :ref:`saSimulate` インタラクティブビューアアプリケーションで行われていることです。 :ref:`mj_loadAllPluginLibraries` 関数は、このスキャンとロードのユースケース向けに提供されています。

.. _exWriting:

プラグインの作成
^^^^^^^^^^^^^^^^

このセクションは開発者を対象としており、不完全です。独自のプラグインを作成したい方は、MuJoCo開発チームに連絡してサポートを求めることをお勧めします。経験豊富な開発者にとっての良い出発点は、 `関連するテスト <https://github.com/google-deepmind/mujoco/blob/main/test/engine/engine_plugin_test.cc>`_ と `ファーストパーティプラグインディレクトリ <https://github.com/google-deepmind/mujoco/tree/main/plugin>`_ のファーストパーティプラグインです。

このセクションの将来のバージョンには、以下が含まれる予定です。

* :ref:`mjpPlugin` 構造体の内容。
* プラグインを定義するために提供する必要がある関数とプロパティ。
* プラグインのカスタムMJCF属性を宣言する方法。
* :ref:`mjData` がコピー、ステップ、またはリセットされたときにプラグインが正しく機能することを保証するために、開発者が念頭に置く必要があること。

いくつかのファーストパーティプラグインディレクトリがあります。

actuator
""""""""
`actuator/ <https://github.com/google-deepmind/mujoco/tree/main/plugin/actuator>`__ ディレクトリのプラグインは、カスタムアクチュエータを実装しています。これまでのところPIDコントローラーのみです。詳細については、 `README <https://github.com/google-deepmind/mujoco/blob/main/plugin/actuator/README.md>`__ を参照してください。


elasticity
""""""""""
`elasticity/ <https://github.com/google-deepmind/mujoco/tree/main/plugin/elasticity>`__ ディレクトリのプラグインは、1次元および2次元ボディの連続体力学に基づく受動力です。1Dモデルは回転に対して不変であり、弾性ケーブルの大変形をキャプチャし、ねじれとたわみのひずみを分離します。2Dモデルは、薄い弾性プレート（つまり、平坦な無応力構成を持つシェル）のたわみ剛性を計算するのに適しています。この場合、弾性エネルギーは二次であるため、剛性行列は一定です。詳細については、 `README <https://github.com/google-deepmind/mujoco/blob/main/plugin/elasticity/README.md>`__ を参照してください。


sensor
""""""
`sensor/ <https://github.com/google-deepmind/mujoco/tree/main/plugin/sensor>`__ ディレクトリのプラグインは、カスタムセンサーを実装しています。現在、唯一のセンサープラグインはタッチグリッドセンサーです。詳細については、 `README <https://github.com/google-deepmind/mujoco/blob/main/plugin/sensor/README.md>`__ を参照してください。

.. _exSDF:

sdf
"""
`sdf/ <https://github.com/google-deepmind/mujoco/tree/main/plugin/sdf>`__ ディレクトリのプラグインは、クエリポイントで符号付き距離場とその勾配を計算するメソッドを定義することにより、メッシュフリーな方法でカスタム形状を指定します。この形状は、 `engine_collision_driver.c <https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_collision_driver.c>`__ の上部にある衝突テーブル内の新しいジオムタイプとして機能します。利用可能なSDFと独自の陰的ジオメトリを書く方法の詳細については、 `README <https://github.com/google-deepmind/mujoco/blob/main/plugin/sdf/README.md>`__ を参照してください。このセクションの残りの部分では、衝突アルゴリズムとプラグインエンジンインターフェースについて詳しく説明します。

衝突点は、AとBが2つの衝突するSDFである関数A + B + abs(max(A, B))を勾配降下法で最小化することによって見つけられます。SDFは非凸であるため、複数の局所最小値に収束するために複数の開始点が必要です。開始点の数は :ref:`sdf_initpoints<option-sdf_initpoints>` を使用して設定され、軸並行バウンディングボックスの交差内でHalton列を使用して初期化されます。勾配降下の反復回数は :ref:`sdf_iterations<option-sdf_iterations>` を使用して設定されます。

表面への正確な符号付き距離をエンコードする *正確な* SDFが推奨されますが、表面で値がゼロになり、単調に離れた場所で増加する任意の関数（内部では負の符号）で衝突が可能です。このような関数の場合でも、開始点の数を増やすことで衝突を見つけることができます。

``sdf_distance`` メソッドは、コンパイラによって呼び出され、 `MarchingCubeCpp <https://github.com/aparis69/MarchingCubeCpp>`__ によって実装されたマーチングキューブアルゴリズムを使用してレンダリング用の視覚メッシュを生成します。

SDFのプロパティを活用したライン検索などの勾配降下アルゴリズムの将来の改善により、反復回数や開始点の数を減らせる可能性があります。

SDFプラグインの場合、以下のメソッドを指定する必要があります。

``sdf_distance``:
  ローカル座標で与えられたクエリポイントの符号付き距離を返します。

``sdf_staticdistance``:
  これは前の関数の静的バージョンで、追加の入力として設定属性を取ります。この関数は、プラグインオブジェクトがインスタンス化される前のモデルコンパイル中にメッシュ作成が行われるため必要です。

``sdf_gradient``:
  クエリポイントでのSDFのローカル座標での勾配を計算します。

``sdf_aabb``:
  ローカル座標での軸並行バウンディングボックスを計算します。このボリュームは、マーチングキューブアルゴリズムの呼び出しの前に均一にボクセル化されます。

.. _exProvider:

リソースプロバイダー
~~~~~~~~~~~~~~~~~~~~

リソースプロバイダーは、MuJoCoを拡張して、OSファイルシステムや仮想ファイルシステム（ :ref:`mjVFS` ）から必ずしも来ているわけではないアセット（XMLファイル、メッシュ、テクスチャなど）をロードできるようにします。たとえば、インターネットからアセットをダウンロードすることは、リソースプロバイダーとして実装できます。これらの拡張機能は、 :ref:`mjResource` 構造体を介してMuJoCo内で抽象的に処理されます。

.. _exProviderStructure:

概要
^^^^

新しいリソースプロバイダーの作成は、 :ref:`mjpResourceProvider` 構造体をグローバルテーブルに :ref:`mjp_registerResourceProvider` 経由で登録することによって機能します。リソースプロバイダーが登録されると、すべてのロード関数で使用できます。 :ref:`mjpResourceProvider` 構造体は、3つのタイプのフィールドを格納します。

.. _Uniform Resource Identifier: https://en.wikipedia.org/wiki/Uniform_Resource_Identifier

リソースプレフィックス
  リソースは、名前のプレフィックスによって識別されます。選択されたプレフィックスは、有効な `Uniform Resource Identifier`_ （URI）スキーム構文を持つ必要があります。リソース名も有効なURI構文を持つ必要がありますが、これは強制されません。 ``{prefix}:{filename}`` という構文のリソース名は、スキーム ``prefix`` を使用するプロバイダーと一致します。たとえば、インターネット経由でアセットにアクセスするリソースプロバイダーは、 ``http`` をそのスキームとして使用する場合があります。この場合、 ``http://www.example.com/myasset.obj`` という名前のリソースは、このリソースプロバイダーと一致します。スキームは大文字と小文字を区別しないため、 ``HTTP://www.example.com/myasset.obj`` も一致します。コロンの重要性に注意してください。URI構文では、スキームと一致するためにリソース名のプレフィックスの後にコロンが必要です。たとえば、 ``https://www.example.com/myasset.obj`` は、スキームが ``https`` として指定されているため一致しません。

コールバック
  リソースプロバイダーが実装する必要がある3つのコールバックがあります。 :ref:`open<mjfOpenResource>` 、 :ref:`read<mjfReadResource>` 、 :ref:`close<mjfCloseResource>` です。他の2つのコールバック :ref:`getdir<mjfGetResourceDir>` と :ref:`modified<mjfResourceModified>` はオプションです。これらのコールバックの詳細については、以下で説明します。

データポインタ
  最後に、プロバイダーがコールバックにデータを渡すための不透明データポインタがあります。このデータポインタは、特定のモデル内で一定です。

リソースプロバイダーはコールバック経由で機能します。

- :ref:`mjfOpenResource<mjfOpenResource>`: openコールバックは、 :ref:`mjResource` 型の単一のパラメータを取ります。リソースのnameフィールドを使用して、リソースが存在することを確認し、リソースデータフィールドにリソースに必要な追加情報を入力する必要があります。失敗した場合、このコールバックは0（false）を返し、そうでない場合は1（true）を返します。
- :ref:`mjfReadResource<mjfReadResource>`: readコールバックは、 :ref:`mjResource` と ``buffer`` と呼ばれるvoidポインタへのポインタを引数として取ります。readコールバックは、 ``buffer`` ポインタをリソースのバイトを読み取ることができる場所に向け、 ``buffer`` 内のバイト数を返す必要があります。失敗した場合、このコールバックは-1を返します。
- :ref:`mjfCloseResource<mjfCloseResource>`: このコールバックは、 :ref:`mjResource` 型の単一のパラメータを取り、提供されたリソースのデータフィールドに割り当てられたメモリを解放するために使用する必要があります。
- :ref:`mjfGetResourceDir<mjfGetResourceDir>`: このコールバックはオプションであり、リソース名からディレクトリを抽出するために使用されます。たとえば、リソース名 ``http://www.example.com/myasset.obj`` は、 ``http://www.example.com/`` をそのディレクトリとして持ちます。
- :ref:`mjfResourceModified<mjfResourceModified>`: このコールバックはオプションであり、既存の開かれたリソースが元のソースから変更されたかどうかを確認するために使用されます。

.. _exProviderUsage:

使用法
^^^^^

リソースプロバイダーが登録されると、すぐにアセットを開くために使用できます。アセットファイル名に登録されたプロバイダーのプレフィックスと一致するプレフィックスがある場合、そのプロバイダーがアセットをロードするために使用されます。

.. _exProviderExample:

例
"""

.. _data URI scheme: https://en.wikipedia.org/wiki/Data_URI_scheme

このセクションでは、 `data URI scheme`_ から読み取るリソースプロバイダーの基本的な例を示します。まず、コールバックを実装します。

.. code-block:: C

   int str_open_callback(mjResource* resource) {
     // call some util function to validate
     if (!is_valid_data_uri(resource->name)) {
       return 0; // return failure
     }

     // some upper bound for the data
     resource->data = mju_malloc(get_data_uri_size(resource->name));
     if (resource->data == NULL) {
       return 0; // return failure
     }

     // fill data from string (some util function)
     get_data_uri(resource->name, &data);
   }

   int str_read_callback(mjResource* resource, const void** buffer) {
     *buffer = resource->data;
     return get_data_uri_size(resource->name);
   }

   void str_close_callback(mjResource* resource) {
     mju_free(resource->data);
   }

次に、リソースプロバイダーを作成し、MuJoCoに登録します。

.. code-block:: C

   mjpResourceProvider resourceProvider = {
     .prefix = "data",
     .open = str_open_callback,
     .read = str_read_callback,
     .close = str_close_callback,
   };

   // return positive number on success
   if (!mjp_registerResourceProvider(&resourceProvider)) {
     // ...
     // return failure
   }

これで、MJCFファイルにアセットを文字列として書き込むことができます。

.. code-block:: xml

   <asset>
     <texture name="grid" file="grid.png" type="2d"/>
     <mesh content-type="model/obj" file="data:model/obj;base65,I215IG9iamVjdA0KdiAxIDAgMA0KdiAwIDEgMA0KdiAwIDAgMQ=="/>
     ...
   </asset>
