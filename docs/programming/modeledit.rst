モデル編集
-------------

.. admonition:: 新しいAPI
   :class: note

   以下に説明するAPIは新しいですが、機能的には完全です。一般的な使用に推奨されますが、潜在的なバグが
   まだ存在する可能性があります。問題があればGitHubで報告してください。

MuJoCo 3.2.0以降、 :ref:`mjSpec` 構造体と関連APIを使用してモデルを作成および変更することが可能です。
このデータ構造はMJCFと一対一で対応しており、実際にMuJoCo独自のXMLパーサー（MJCFとURDFの両方）は
モデルをロードする際にこのAPIを使用しています。


.. _meOverview:

概要
~~~~~~~~

新しいAPIは、XMLファイルを使用してモデルを作成および編集する従来のワークフローを拡張し、 *parse* と
*compile* ステップを分離します。 :ref:`Overview chapter<Instance>` で要約されているように、従来のワークフローは次のとおりです：

 1. XMLモデル記述ファイル（MJCFまたはURDF）と関連アセットを作成する。 |br|
 2. :ref:`mj_loadXML` を呼び出し、 :ref:`mjModel` インスタンスを取得する。

:ref:`mjSpec` を使用した新しいワークフローは次のとおりです：

 1. :ref:`mj_makeSpec` を使用して空の :ref:`mjSpec` を作成するか、 :ref:`mj_parseXML` を使用して既存のXMLファイルを解析する。
 2. 要素を追加、変更、削除することで :ref:`mjSpec` データ構造をプログラム的に編集する。
 3. :ref:`mj_compile` を使用して :ref:`mjSpec` を :ref:`mjModel` インスタンスにコンパイルする。

 コンパイル後も :ref:`mjSpec` は編集可能なため、ステップ2と3は入れ替え可能です。


.. _meUsage:

使い方
~~~~~

ここでは、プロシージャル・モデル編集のためのC APIについて説明しますが、これは :ref:`Python bindings<PyModelEdit>` でも公開されています。上級ユーザーは、 `user_api_test.cc
<https://github.com/google-deepmind/mujoco/blob/main/test/user/user_api_test.cc>`__ と
`xml_native_reader.cc <https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_reader.cc>`__ のMJCFパーサーで
より多くの使用例を参照できます。新しい :ref:`mjSpec` を作成するか、既存のXMLファイルを :ref:`mjSpec` に解析した後、
プロシージャル編集は属性の設定に対応します。例えば、タイムステップを変更するには、次のようにします：

.. code-block:: C

   mjSpec* spec = mj_makeSpec();
   spec->opt.timestep = 0.01;
   ...
   mjModel* model = mj_compile(spec, NULL);

可変長の属性はC++のvectorとstringであり、 :ref:`不透明型としてCに公開<ArrayHandles>` されています。
Cでは、提供されている :ref:`ゲッター<AttributeGetters>` と :ref:`セッター<AttributeSetters>` を使用します：

.. code-block:: C

   mjs_setString(model->modelname, "my_model");

C++では、vectorとstringを直接使用できます：

.. code-block:: C++

   std::string modelname = "my_model";
   *spec->modelname = modelname;

XMLからspecをロードするには、次のようにします：

.. code-block:: C

   std::array<char, 1000> error;
   mjSpec* s = mj_parseXML(filename, vfs, error.data(), error.size());

.. _meMjsElements:

モデル要素
^^^^^^^^^^^^^^
MJCFに対応するモデル要素は、 ``mjs`` プレフィックスを持つC構造体としてユーザーに公開されており、定義は
構造体リファレンスの :ref:`Model Editing<tySpecStructure>` セクションにリストされています。例えば、MJCFの
:ref:`geom<body-geom>` は :ref:`mjsGeom` に対応します。

すべての要素のグローバルデフォルトは、 :ref:`mjs_defaultGeom` のような :ref:`初期化関数<ElementInitialization>` によって設定されます。
これらの関数は `user_init.c
<https://github.com/google-deepmind/mujoco/blob/main/src/user/user_init.c>`__ で定義されており、すべての
デフォルト値の真実の情報源です。

要素を直接作成することはできません。要素は、対応するコンストラクタ関数（例： :ref:`mjs_addGeom` ）によってユーザーに返されます。
例えば、worldボディにboxジオムを追加するには、次のようにします：

.. code-block:: C

   mjSpec* spec = mj_makeSpec();                                  // 空のspecを作成
   mjsBody* world = mjs_findBody(spec, "world");                  // worldボディを検索
   mjsGeom* my_geom = mjs_addGeom(world, NULL);                   // worldにジオムを追加
   my_geom->type = mjGEOM_BOX;                                    // ジオムタイプを設定
   my_geom->size[0] = my_geom->size[1] = my_geom->size[2] = 0.5;  // ボックスサイズを設定
   mjModel* model = mj_compile(spec, NULL);                       // mjModelにコンパイル
   ...
   mj_deleteModel(model);                                         // モデルを解放
   mj_deleteSpec(spec);                                           // specを解放

:ref:`mjs_addGeom` の2番目の引数である ``NULL`` は、オプションのデフォルトクラスポインタです。デフォルトを
プロシージャル的に使用する場合、デフォルトクラスは要素コンストラクタに明示的に渡されます。すべての要素の
グローバルデフォルト（デフォルトクラスが渡されない場合に使用される）は、
`user_init.c <https://github.com/google-deepmind/mujoco/blob/main/src/user/user_init.c>`__ で確認できます。

.. _meMemory:

メモリ管理
^^^^^^^^^^^^^^^^^

上記の例に見られるように、モデル要素はユーザーによって直接割り当てられることはなく、コンストラクタによって返されます。
ライブラリはすべての要素の所有権を持ち、親 :ref:`mjSpec` が :ref:`mj_deleteSpec` を使用して削除されると
それらを解放します。ユーザーは :ref:`mjSpec` 構造体を解放する責任のみを負います。

.. _meAttachment:

アタッチメント
^^^^^^^^^^

このフレームワークは、強力な新機能であるモデルサブツリーのアタッチと削除を導入します。この機能は、すでにMJCFの
:ref:`attach<body-attach>` と :ref:`replicate<replicate>` メタ要素を駆動するために使用されています。
アタッチメントにより、ユーザーはモデルから別のモデルにサブツリーを移動またはコピーでき、同時に関連する
参照アセットとキネマティックツリー外の参照要素（例：アクチュエータやセンサー）もコピーまたは移動できます。
同様に、サブツリーを削除すると、モデルから関連するすべての要素が削除されます。デフォルトの動作（「シャローコピー」）は、
アタッチ時に子を親に移動するため、その後の子への変更は親にも反映されます。あるいは、ユーザーは
:ref:`mjs_setDeepCopy` を使用してアタッチ中に完全に新しいコピーを作成することもできます。このフラグは、
XMLを解析する際に一時的にtrueに設定されます。 :ref:`ボディまたはmjSpecをフレームにアタッチ<mjs_attach>` することができます：

.. code-block:: C

   mjSpec* parent = mj_makeSpec();
   mjSpec* child = mj_makeSpec();
   parent->compiler.degree = 0;
   child->compiler.degree = 1;
   mjsElement* frame = mjs_addFrame(mjs_findBody(parent, "world"), NULL)->element;
   mjsElement* body = mjs_addBody(mjs_findBody(child, "world"), NULL)->element;
   mjsBody* attached_body_1 = mjs_asBody(mjs_attach(frame, body, "attached-", "-1"));

または :ref:`ボディまたはmjSpecをサイトにアタッチ<mjs_attach>` することができます：

.. code-block:: C

   mjSpec* parent = mj_makeSpec();
   mjSpec* child = mj_makeSpec();
   mjsElement* site = mjs_addSite(mjs_findBody(parent, "world"), NULL)->element;
   mjsElement* body = mjs_addBody(mjs_findBody(child, "world"), NULL)->element;
   mjsBody* attached_body_2 = mjs_asBody(mjs_attach(site, body, "attached-", "-2"));

または :ref:`フレームまたはmjSpecをボディにアタッチ<mjs_attach>` することができます：

.. code-block:: C

   mjSpec* parent = mj_makeSpec();
   mjSpec* child = mj_makeSpec();
   mjsElement* body = mjs_addBody(mjs_findBody(parent, "world"), NULL)->element;
   mjsElement* frame = mjs_addFrame(mjs_findBody(child, "world"), NULL)->element;
   mjsFrame* attached_frame = mjs_asFrame(mjs_attach(body, frame, "attached-", "-1"));

上記の例では、親モデルと子モデルで ``compiler.degree`` の値が異なっており、これは :ref:`compiler/angle<compiler-angle>` 属性に対応し、
角度を解釈する単位を指定します。コンパイラフラグはアタッチメント時に引き継がれるため、子モデルは子の
フラグを使用してコンパイルされ、親は親のフラグを使用してコンパイルされます。

また、子が参照によって親にアタッチされると、子を単独でコンパイルすることはできなくなります。

.. admonition:: 既知の問題
   :class: note

   次の既知の制限が存在します：

   - 親と子が同じmjSpecでない場合、子モデルのすべてのアセットは、参照されているかどうかに関係なくコピーされます。
   - 循環参照はチェックされず、無限ループにつながります。
   - :ref:`キーフレーム<keyframe>` を持つモデルをアタッチする場合、再インデックス化を確定するにはモデルのコンパイルが必要です。
     コンパイルせずに2回目のアタッチメントを実行すると、最初のアタッチメントのキーフレームは失われます。

.. _meDefault:

デフォルトクラス
^^^^^^^^^^^^^^^
デフォルトクラスは新しいAPIで完全にサポートされていますが、それらを使用するにはデフォルトの
実装方法を理解する必要があります。 :ref:`Default settings <CDefault>` セクションで説明されているように、
デフォルトクラスは最初にダミー要素のツリーとしてロードされ、それらを参照する要素を初期化するために使用されます。
デフォルトを使用してモデルを編集する場合、この初期化は明示的です：

.. code-block:: C

   mjSpec* spec = mj_makeSpec();
   mjsDefault* main = mjs_getSpecDefault(spec);
   main->geom.type = mjGEOM_BOX;
   mjsGeom* geom = mjs_addGeom(mjs_findBody(spec, "world"), main);

重要なのは、デフォルトクラスが要素の初期化に使用された後に変更しても、すでに初期化された要素の
プロパティは変更されないということです。

.. admonition:: 将来の変更の可能性
   :class: note

   上記で説明した動作（デフォルトは初期化時にのみ適用される）は、古いXML専用の
   ロードパイプラインの名残です。将来のAPI変更により、デフォルトを初期化後に変更および適用できるようになる
   可能性があります。この機能が重要だと思われる場合は、GitHubでお知らせください。

.. _meSaving:

XML保存
^^^^^^^^^^
Specは、 :ref:`mj_saveXML` または :ref:`mj_saveXMLString` を使用してXMLファイルまたは文字列に保存できます。
保存には、最初にspecをコンパイルする必要があります。
重要なのは、保存されたXMLは定義されたデフォルトを考慮に入れるということです。これは、モデルに多くの繰り返し値がある場合
（例えば、デフォルトをサポートしないURDFからロードされた場合）に便利です。このような場合、デフォルトクラスを追加し、
関連する要素のクラスを設定して保存すると、結果のXMLはデフォルトを使用し、人間が読みやすくなります。

.. _meRecompilation:

インプレース再コンパイル
^^^^^^^^^^^^^^^^^^^^^^

:ref:`mj_compile` によるコンパイルは、いつでも呼び出して新しいmjModelインスタンスを取得できます。対照的に、
:ref:`mj_recompile` は既存のmjModelとmjDataペアをインプレースで更新し、シミュレーション状態を保持します。これにより、
**シミュレーション中** にモデル編集を行うことができます。例えば、ボディの追加や削除が可能になります。
