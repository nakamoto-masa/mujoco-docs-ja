mjcPhysics
==========

.. WARNING:: OpenUSDサポートは現在実験的なものであり、頻繁に変更される可能性があります。

``mjcPhysics`` `スキーマ <https://openusd.org/release/api/_usd__page__generating_schemas.html>`__ により、USDファイル内で直接MuJoCoシミュレーション環境の詳細な仕様を指定できます。目的は `UsdPhysics
<https://openusd.org/release/api/usd_physics_page_front.html>`__ を置き換えることではなく、既存の概念を拡張し、必要な場合にのみ新しい型を作成することです。

このスキーマは `コードレス <https://openusd.org/dev/api/_usd__page__generating_schemas.html#Codeless_Schemas>`__ で使用することも、C++バインディングでビルドすることもできます。MuJoCo内部で使用するために `usdGenSchema
<https://openusd.org/dev/api/_usd__page__generating_schemas.html>`_ を介して `コード
<https://github.com/google-deepmind/mujoco/tree/main/src/experimental/usd/mjcPhysics>`__ を事前生成していますが、MuJoCo外でも動作するはずです。

APIスキーマ
-----------

MjcSceneAPI
^^^^^^^^^^^

このAPIスキーマは、MuJoCoシミュレーションのグローバルオプションを提供します。MJCFの ``<option>``、 ``<option/flag>``、 ``<compiler>`` 要素を統合したものです。ユーザーは、これを既存の
`UsdPhysicsScene <https://openusd.org/dev/api/class_usd_physics_scene.html>`__
primに適用する必要があります。

主な属性は以下の通りです：

-   **mjc:option**: この名前空間の属性は ``<option>`` 要素にマッピングされます。
-   **mjc:flag**: この名前空間の属性は ``<option/flag>`` 要素にマッピングされます。
-   **mjc:compiler**: この名前空間の属性は ``<compiler>`` 要素にマッピングされます。

MjcSiteAPI
^^^^^^^^^^

このAPIクラスはMuJoCoサイトを定義するために使用され、
`UsdGeomSphere <https://openusd.org/dev/api/class_usd_geom_sphere.html>`__、
`UsdGeomCapsule <https://openusd.org/dev/api/class_usd_geom_capsule.html>`__、
`UsdGeomCylinder <https://openusd.org/dev/api/class_usd_geom_cylinder.html>`__、および
`UsdGeomCube <https://openusd.org/dev/api/class_usd_geom_cube.html>`__ に適用できます。

MjcImageableAPI
^^^^^^^^^^^^^^^

このAPIクラスは、MuJoCoにおける純粋に視覚的なエンティティの属性を提供します。MuJoCo用語では、これらは ``contype = conaffinity = 0`` を持つものとして定量化されます。

MjcCollisionAPI
^^^^^^^^^^^^^^^

このAPIクラスは、コリジョンジオメトリを表すprimに適用され、
`UsdPhysicsCollisionAPI <https://openusd.org/dev/api/class_usd_physics_collision_a_p_i.html>`__ と併用する必要があります。

MjcMeshCollisionAPI
^^^^^^^^^^^^^^^^^^^

このAPIクラスは、メッシュコリジョンジオメトリを表すprimに適用され、
`UsdPhysicsMeshCollisionAPI <https://openusd.org/dev/api/class_usd_physics_mesh_collision_a_p_i.html>`__ と併用する必要があります。

MjcJointAPI
^^^^^^^^^^^

このAPIクラスは `UsdPhysicsJoint <https://openusd.org/dev/api/class_usd_physics_joint.html>`__ primに適用され、MuJoCoジョイントを完全に記述するための追加属性を提供します。

MjcMaterialAPI
^^^^^^^^^^^^^^

このAPIクラスは物理マテリアルの属性を提供し、`UsdPhysicsMaterialAPI
<https://openusd.org/dev/api/class_usd_physics_material_a_p_i.html>`__ の拡張です。

型スキーマ
------------

MjcActuator
^^^^^^^^^^^

このクラスはMuJoCoアクチュエータを表し、 `リレーションシップ <https://openusd.org/dev/api/class_usd_relationship.html>`__ を介して指定された伝達対象のジョイント、ボディ、またはサイトに力を適用する役割を担います。

既存の `UsdPhysicsDriveAPI <https://openusd.org/dev/api/class_usd_physics_drive_a_p_i.html>`__ は使用しません。これは実行時の構造に近く、概念が非常に異なるためです。

MjcKeyframe
^^^^^^^^^^^

この型は、特定の時刻におけるシミュレータの状態を表すテンソル値を保持します。

MJCFでは、これは ``<keyframe>`` 要素であり、 ``time`` 属性を持ちます。USDでは、time属性を代わりに
`timeSamples <https://openusd.org/release/tut_xforms.html>`__ にマッピングします。

キーフレーム内の値の順序は、合成されたステージ内の剛体の深さ優先順序探索にマッピングする必要があります。

MjcTendon
^^^^^^^^^

この型は、固定テンドンと空間テンドンの両方を表します。

MJCFでは、これは ``<tendon>`` 要素です。テンドンのパスは、 ``mjc:path`` リレーションシップ属性内のターゲットの順序付きリストによって表されます。MJCFでは、パスターゲットに ``sidesite`` や ``divisor`` などの属性を指定できますが、USDではリレーションシップ属性にデータをエレガントに添付できないため、これらは ``mjc:sideSites`` や ``mjs:path:divisors`` などのインデックス付き配列属性になります。
