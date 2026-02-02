流体力
============

適切な流体力学のシミュレーションはMuJoCoの範囲を超えており、私たちが支援しようとするアプリケーションには遅すぎます。それでも、飛行や泳ぎといった動作をシミュレートするのに十分な2つの現象論的モデルを提供しています。これらのモデルは *状態を持たない* という意味で、周囲の流体に追加の状態が割り当てられることなく、流体媒体を通して移動する剛体の顕著な特徴を捉えることができます。

両方のモデルは、 :ref:`density<option-density>` と :ref:`viscosity<option-viscosity>` 属性を正の値に設定することで有効になります。これらのパラメータは、媒体の密度 :math:`\rho` と粘性 :math:`\beta` に対応します。

1. :ref:`慣性ベースモデル<flInertia>` は、粘性と密度のみを使用し、ボディの等価慣性ボックスから形状を推測します。
2. :ref:`楕円体ベースモデル <flEllipsoid>` はより精巧で、ジオムの楕円体近似を使用します。
   媒体の全体的な粘性と密度に加えて、このモデルは相互作用するジオムごとに5つの調整可能なパラメータを公開します。

.. tip::
   :ref:`数値積分<geIntegration>` セクションで詳述されているように、陰的積分は速度依存力の存在下でシミュレーションの安定性を大幅に改善します。以下に説明する両方の流体力モデルはこの特性を示すため、流体力を使用する場合は ``implicit`` または ``implicitfast`` :ref:`積分器<option-integrator>` が推奨されます。両方のモデルに必要な解析的導関数は完全に実装されています。

.. _flInertia:

慣性モデル
-------------

このモデルでは、流体力学の目的における各ボディの形状は *等価慣性ボックス* であると仮定されます。これは可視化することもできます。質量 :math:`\mathcal{M}` と慣性行列 :math:`\mathcal{I}` を持つボディについて、等価慣性ボックスの半寸法（つまり、半幅、半奥行き、半高さ）は次のようになります。

.. math::
   \begin{align*}
   r_x = \sqrt{\frac{3}{2 \mathcal{M}} \left(\mathcal{I}_{yy} + \mathcal{I}_{zz} - \mathcal{I}_{xx} \right)}  \\
   r_y = \sqrt{\frac{3}{2 \mathcal{M}} \left(\mathcal{I}_{zz} + \mathcal{I}_{xx} - \mathcal{I}_{yy} \right)}  \\
   r_z = \sqrt{\frac{3}{2 \mathcal{M}} \left(\mathcal{I}_{xx} + \mathcal{I}_{yy} - \mathcal{I}_{zz} \right)}
   \end{align*}

:math:`\mathbf{v}` と :math:`\boldsymbol{\omega}` をボディローカルフレーム（等価慣性ボックスと整列）におけるボディの線速度と角速度とします。流体が固体に及ぼす力 :math:`\mathbf{f}_{\text{inertia}}` とトルク :math:`\mathbf{g}_{\text{inertia}}` は次の項の和です。

.. math::
   \begin{align*}
   \mathbf{f}_{\text{inertia}} &= \mathbf{f}_D + \mathbf{f}_V  \\
   \mathbf{g}_{\text{inertia}} &= \mathbf{g}_D + \mathbf{g}_V
   \end{align*}

ここで、添字 :math:`D` と :math:`V` は二次抗力（Drag）と粘性抵抗（Viscous resistance）を表します。

二次抗力項は流体の密度 :math:`\rho` に依存し、ボディの速度の2乗に比例してスケールし、高レイノルズ数での流体力の妥当な近似です。
トルクは、回転によって生じる力を表面積にわたって積分することで得られます。
力とトルクの :math:`i` 番目の成分は次のように書けます。

.. math::
   \begin{aligned}
   f_{D, i} = \quad &- 2  \rho r_j r_k |v_i| v_i \\
   g_{D, i} = \quad &- {1 \over 2} \rho r_i \left(r_j^4 + r_k^4 \right) |\omega_i| \omega_i \\
   \end{aligned}

粘性抵抗項は流体の粘性 :math:`\beta` に依存し、ボディの速度に線形にスケールし、低レイノルズ数での流体力を近似します。粘性は密度とは独立して使用でき、シミュレーションをより減衰させることができます。低レイノルズ数での等価球体の公式を使用します。等価球体の半径は
:math:`r_{eq} = (r_x + r_y + r_z) / 3` です。その結果得られるローカルボディ座標系での3D力とトルクは次のようになります。

.. math::
   \begin{aligned}
   f_{V, i} = \quad &- 6 \beta \pi r_{eq} v_i \\
   g_{V, i} = \quad &- 8 \beta \pi r_{eq}^3 \omega_i \\
   \end{aligned}

これらの力は、ゼロでない :ref:`wind<option-wind>` を指定することによっても影響を受けます。これは流体力学計算においてボディの線速度から減算される3Dベクトルです。

.. _flEllipsoid:

楕円体モデル
---------------

.. cssclass:: caption-small
.. figure:: ../images/computation/fruitfly.png
   :figwidth: 50%
   :align: right

   この図の飛行可能なキイロショウジョウバエ（Drosophila Melanogaster）モデルは :cite:t:`Vaxenburg2024` に記載されています。


このセクションでは、ジオム形状の楕円体近似に基づいた、周囲の流体によって移動する剛体に及ぼされる力の状態を持たないモデルを説明し導出します。このモデルは、前のセクションの慣性ベースモデルよりも、さまざまなタイプの流体力のきめ細かな制御を提供します。このモデルの動機となる使用例は昆虫の飛行です。右の図を参照してください。

要約
~~~~~~~

モデルは :ref:`fluidshape<body-geom-fluidshape>` 属性を ``ellipsoid`` に設定することで、ジオムごとに有効化されます。これにより、親ボディの慣性ベースモデルも無効になります。
:ref:`fluidcoef<body-geom-fluidcoef>` 属性の5つの数値は、次の意味に対応します。


.. list-table::
   :width: 60%
   :align: left
   :widths: 1 5 2 1
   :header-rows: 1

   * - インデックス
     - 説明
     - 記号
     - デフォルト
   * - 0
     - 鈍い抗力係数
     - :math:`C_{D, \text{blunt}}`
     - 0.5
   * - 1
     - 細長い抗力係数
     - :math:`C_{D, \text{slender}}`
     - 0.25
   * - 2
     - 角度抗力係数
     - :math:`C_{D, \text{angular}}`
     - 1.5
   * - 3
     - クッタ揚力係数
     - :math:`C_K`
     - 1.0
   * - 4
     - マグヌス揚力係数
     - :math:`C_M`
     - 1.0

モデルの要素は :cite:t:`andersen2005b` を3次元に一般化したものです。
流体が固体に及ぼす力 :math:`\mathbf{f}_{\text{ellipsoid}}` とトルク
:math:`\mathbf{g}_{\text{ellipsoid}}` は次の項の和です。

.. math::
   \begin{align*}
   \mathbf{f}_{\text{ellipsoid}} &= \mathbf{f}_A + \mathbf{f}_D + \mathbf{f}_M + \mathbf{f}_K + \mathbf{f}_V  \\
   \mathbf{g}_{\text{ellipsoid}} &= \mathbf{g}_A + \mathbf{g}_D + \mathbf{g}_V
   \end{align*}

ここで、添字 :math:`A`、 :math:`D`、 :math:`M`、 :math:`K`、 :math:`V` は、付加質量（Added mass）、粘性抗力（viscous Drag）、マグヌス揚力（Magnus lift）、
クッタ揚力（Kutta lift）、粘性抵抗（Viscous resistance）をそれぞれ表します。 :math:`D`、 :math:`M`、 :math:`K` の項は、上記のそれぞれの
:math:`C_D`、 :math:`C_M`、 :math:`C_K` 係数でスケールされ、粘性抵抗は流体の粘性
:math:`\beta` でスケールされますが、付加質量項はスケールできません。

表記法
~~~~~~~~

非粘性、非圧縮性の静止流体中の物体の運動を密度 :math:`\rho` で記述します。任意の形状の物体は、モデルでは半軸
:math:`\mathbf{r} = \{r_x, r_y, r_z\}` の等価楕円体として記述されます。
問題は、楕円体の辺と整列し、楕円体とともに移動する参照フレームで記述されます。ボディは速度 :math:`\mathbf{v} = \{v_x, v_y, v_z\}` と角速度
:math:`\boldsymbol{\omega} = \{\omega_x, \omega_y, \omega_z\}` を持ちます。また、次の記号も使用します。

.. math::
   \begin{align*}
       r_\text{max} &= \max(r_x, r_y, r_z) \\
       r_\text{min} &= \min(r_x, r_y, r_z) \\
       r_\text{mid} &= r_x + r_y + r_z - r_\text{max} - r_\text{min}
   \end{align*}

レイノルズ数は、流れ内の慣性力と粘性力の比であり、 :math:`Re=u~l/\beta` と定義されます。ここで、
:math:`\beta` は流体の動粘性係数、 :math:`u` は流れの特性速度（またはフレーム変更により、ボディの速度）、 :math:`l` は流れまたはボディの特性サイズです。

:math:`\Gamma` を循環を表すために使用します。これは、閉じた曲線周りの速度場の線積分
:math:`\Gamma = \oint \mathbf{v} \cdot \textrm{d} \mathbf{l}` であり、ストークスの定理により、
:math:`\Gamma = \int_S \nabla \times \mathbf{v} \cdot \textrm{d}\mathbf{s}` となります。
流体力学の表記法では、記号 :math:`\boldsymbol{\omega}` は角速度ではなく、
:math:`\nabla \times \mathbf{v}` として定義される渦度にしばしば使用されます。剛体運動の場合、渦度は角速度の2倍です。

最後に、添字 :math:`i, j, k` を使用して、 :math:`x, y, z` に対称に適用される3つ組の方程式を表します。例えば、 :math:`a_i = b_j + b_k` は次の3つの方程式の略記です。

.. math::
   \begin{align*}
       a_x &= b_y + b_z \\
       a_y &= b_x + b_z \\
       a_z &= b_x + b_y
   \end{align*}

.. _flProjection:

楕円体の投影
~~~~~~~~~~~~~~~~~~~~

次の結果を示します。

.. admonition:: 補題
   :class: note

   座標軸 :math:`(x, y, z)` と整列した半軸 :math:`(r_x, r_y, r_z)` を持つ楕円体と、単位ベクトル :math:`\mathbf{u} = (u_x, u_y, u_z)` が与えられた場合、楕円体が
   :math:`\mathbf{u}` に垂直な平面上に投影される面積は次のようになります。

   .. math::
      A^{\mathrm{proj}}_{\mathbf{u}} = \pi \sqrt{\frac{r_y^4 r_z^4 u_x^2 + r_z^4 r_x^4 u_y^2 + r_x^4 r_y^4 u_z^2}{r_y^2 r_z^2 u_x^2 + r_z^2 r_x^2 u_y^2 + r_x^2 r_y^2 u_z^2}}

.. collapse:: 導出を展開

   .. admonition:: 補題の導出
      :class: tip

      **楕円の面積**
         原点を中心とする任意の楕円は、二次形式 :math:`\mathbf{x}^T Q \mathbf{x} = 1` で記述できます。ここで、 :math:`Q` は楕円の向きと半軸長を定義する実対称正定値2x2行列であり、 :math:`\mathbf{x} = (x, y)` は楕円上の点です。楕円の面積は次のように与えられます。

         .. math::
            A = \frac{\pi}{\sqrt{\det Q}} .

      **楕円体の断面**
         まず、原点を中心とする楕円体と、単位法線 :math:`\mathbf{n} = (n_x, n_y, n_z)` を持つ原点を通る平面 :math:`\Pi_{\mathbf{n}}` との交差によって形成される楕円の面積を計算します。 :math:`(r_x, r_y, r_z)` を楕円体の半軸長とします。一般性を失うことなく、楕円体の軸が座標軸と整列していると仮定するだけで十分です。楕円体は
         :math:`\mathbf{x}^T Q \mathbf{x} = 1` として記述できます。ここで、
         :math:`Q = \textrm{diag}\mathopen{}\left( \left. 1 \middle/ r_x^2 \right., \left. 1 \middle/ r_y^2 \right., \left. 1 \middle/ r_z^2 \right. \right)\mathclose{}`
         であり、 :math:`\mathbf{x} = (x, y, z)` は楕円体上の点です。

         平面 :math:`\Pi_{\mathbf{n}}` を楕円体と一緒に回転させて、回転した平面の法線が :math:`z` 軸に沿って向くようにします。これにより、 :math:`z` 座標をゼロに設定することで望ましい交差を得ることができます。 :math:`z` 軸に沿った単位ベクトルを :math:`\mathbf{\hat{z}}` とすると、次のようになります。

         .. math::
            \begin{align*}
            \mathbf{n} \times \mathbf{\hat{z}} &= \sin\theta \, \mathbf{m}, \\
            \mathbf{n} \cdot \mathbf{\hat{z}} &= \cos\theta ,
            \end{align*}

         ここで、 :math:`\mathbf{m}` は回転軸を定義する単位ベクトルであり、 :math:`\theta` は回転角です。これらを並べ替えて、回転クォータニオンを形成するために必要な量、すなわち

         .. math::
            \begin{align*}
            \cos\frac{\theta}{2}
            &= \sqrt{\frac{1+\cos\theta}{2}}
            &= \sqrt{\frac{1 + \mathbf{n} \cdot \mathbf{\hat{z}}}{2}}, \\
            \sin\frac{\theta}{2}\,\mathbf{m}
            &= \frac{\mathbf{n} \times \mathbf{\hat{z}}}{2\cos\frac{\theta}{2}}
            &= \frac{\mathbf{n} \times \mathbf{\hat{z}}}{\sqrt{2 (1 + \mathbf{n} \cdot \mathbf{\hat{z}})}} .
            \end{align*}

         を得ます。回転クォータニオン :math:`q = q_r + q_x \mathbf{i} + q_y \mathbf{j} + q_z \mathbf{k}` は次のように与えられます。

         .. math::
            q_r = \sqrt{\frac{1 + n_z}{2}}, \quad
            q_x = \frac{n_y}{\sqrt{2 \left(1+n_z\right)}},  \quad
            q_y = \frac{-n_x}{\sqrt{2 \left(1+n_z\right)}}, \quad
            q_z = 0 .

         これから、回転行列は次のように与えられます。

         .. math::
            \def\arraystretch{1.33}
            \begin{align*}
            R &= \begin{pmatrix}
            1 - 2 q_y^2 - 2 q_z^2 & 2 \left(q_x q_y - q_r q_z\right) & 2 \left(q_x q_z + q_r q_y\right) \\
            2 \left(q_x q_y + q_r q_z\right) & 1 - 2 q_x^2 - 2 q_z^2 & 2 \left(q_y q_z - q_r q_x\right) \\
            2 \left(q_x q_z - q_r q_y\right) & 2 \left(q_y q_z + q_r q_x\right) & 1 - 2 q_x^2 - 2 q_y^2
            \end{pmatrix} \\
            &= \begin{pmatrix}
            1 - \left. n_x^2 \middle/ \left( 1+n_z \right) \right. & \left. -n_x n_y \middle/ \left( 1+n_z \right) \right. & -n_x \\
            \left. -n_x n_y \middle/ \left( 1+n_z \right) \right. & 1 - \left. n_y^2 \middle/ \left( 1+n_z \right) \right. & -n_y \\
            n_x & n_y & 1 - \left. \left( n_x^2 + n_y^2 \right) \middle/ \left( \vphantom{n_x^2} 1+n_z \right) \right.
            \end{pmatrix},
            \end{align*}

         そして、回転した楕円体は変換された二次形式を介して記述されます。

         .. math::
            \mathbf{x}^T Q' \mathbf{x} = \mathbf{x}^T \left( R^T Q R \right) \mathbf{x} = 1 .

         上記の楕円面積の公式から、 :math:`z=0` での楕円の面積には、次が必要です。

         .. math::
            \begin{align*}
            Q'_{xx} &= \frac{1}{r_x^2} R_{xx}^2 + \frac{1}{r_y^2} R_{yx}^2 + \frac{1}{r_z^2} R_{zx}^2 , \\
            Q'_{yy} &= \frac{1}{r_x^2} R_{xy}^2 + \frac{1}{r_y^2} R_{yy}^2 + \frac{1}{r_z^2} R_{zy}^2 , \\
            Q'_{xy} &= \frac{1}{r_x^2} R_{xx} R_{xy} + \frac{1}{r_y^2} R_{yx} R_{yy} + \frac{1}{r_z^2} R_{zx} R_{zy} ,
            \end{align*}

         そして、望ましい面積は次のように与えられます。

         .. math::
            A^{\cap}_{\mathbf{n}}
            = \frac{\pi}{\sqrt{\vphantom{Q'^2_{xy}} \det Q'}}
            = \frac{\pi}{\sqrt{Q'_{xx} Q'_{yy} - Q'^2_{xy}}}
            = \frac{\pi r_x r_y r_z}{\sqrt{r_x^2 n_x^2 + r_y^2 n_y^2 + r_z^2 n_z^2}},

         ここで、上付き文字 :math:`\cap` は、面積が :math:`\Pi_{\mathbf{n}}` との *交差* の楕円に関するものであることを示します。

      **投影楕円**
         :math:`\mathbf{u} = (u_x, u_y, u_z)` を何らかの単位ベクトル（この文脈では、楕円体に衝突する流体の速度の方向）とし、 :math:`\Pi_{\mathbf{u}}` を :math:`\mathbf{u}` に垂直な平面とします。
         一般に、楕円体 :math:`\mathcal{E}` を :math:`\Pi_{\mathbf{u}}` に投影することによって形成される楕円
         （ :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}}` と表記）は、 :math:`\mathcal{E}` と :math:`\Pi_{\mathbf{u}}` を交差させることによって形成される楕円
         （ :math:`\mathcal{E}^{\cap}_{\mathbf{u}}` と表記）とは異なります。

         :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}}` の重要な性質は、 :math:`\mathbf{u}` が
         :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}}` 上のすべての点で楕円体 :math:`\mathcal{E}` に接していることです。

         :math:`\mathcal{E}` を、伸縮変換 :math:`T = \mathrm{diag}(r_x, r_y, r_z)` による単位球 :math:`\mathcal{S}` の像とみなすことができます。さらに、 :math:`\mathbf{\tilde{u}}` が
         :math:`\mathcal{S}` に接するベクトルである場合、その像
         :math:`\mathbf{u}=T\mathbf{\tilde{u}}=(r_x \tilde{u}_x, r_y \tilde{u}_y, r_z \tilde{u}_z)` は楕円体に接します。楕円 :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}}` は、したがって、 :math:`\mathcal{S}` と :math:`\Pi_{\mathbf{\tilde{u}}}` の交差における円 :math:`\mathcal{C}^{\cap}_{\mathbf{\tilde{u}}}` の :math:`T` による像です（球の場合、 :math:`\mathcal{C}^{\cap}` と
         :math:`\mathcal{C}^{\mathrm{proj}}` は一致します）。

         :math:`\mathbf{\tilde{v}}` と :math:`\mathbf{\tilde{w}}` を平面
         :math:`\Pi_{\mathbf{\tilde{u}}}` 内の何らかの直交ベクトルの対とすると、 :math:`\mathbf{\tilde{u}} = \mathbf{\tilde{v}} \times \mathbf{\tilde{w}}` です。
         それらの :math:`T` による像は :math:`\mathbf{v} = (r_x \tilde{v}_x, r_y \tilde{v}_y, r_z \tilde {v}_z)` と
         :math:`\mathbf{w} = (r_x \tilde{w}_x, r_y \tilde{w}_y, r_z \tilde {w}_z)` であり、それらは
         :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}}` の平面内の直交ベクトルのままです。楕円
         :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}}` への（単位でない）法線は、したがって次のように与えられます。

         .. math::
            \mathbf{N} = \mathbf{v} \times \mathbf{w}
            = (r_y r_z \tilde{u}_x, r_z r_x \tilde{u}_y, r_x r_y \tilde{u}_z)
            = \left( \frac{r_y r_z}{r_x} u_x, \frac{r_z r_x}{r_y} u_y, \frac{r_x r_y}{r_z} u_z \right).

         これは、 :math:`\mathcal{E}^{\mathrm{proj}}_{\mathbf{u}} = \mathcal{E}^{\cap}_{\mathbf{n}}` を示しており、ここで
         :math:`\mathbf{n} = \mathbf{N} / \left\Vert\mathbf{N}\right\Vert` です。その面積は、前のセクションで導出された公式によって与えられ、上記に示された結果につながります。

付加質量
~~~~~~~~~~

流体中を移動するボディについて、付加質量または仮想質量は、ボディの動きによって移動される流体の慣性を測定します。これはポテンシャル流理論から導出できます（つまり、非粘性流にも存在します）。

:cite:t:`lamb1932` の第5章に従って、静止状態から流体内に運動を生成することによって移動するボディに及ぼされる力 :math:`\mathbf{f}_{V}` とトルク :math:`\mathbf{g}_{V}` は次のように書けます。

.. math::
   \begin{align*}
       \mathbf{f}_{A} &= - \frac{\textrm{d}}{\textrm{d} t} \nabla_{\mathbf{v}} \mathcal{T} + \nabla_{\mathbf{v}} \mathcal{T} \times \boldsymbol{\omega} \\
       \mathbf{g}_{A} &= - \frac{\textrm{d}}{\textrm{d} t} \nabla_{\boldsymbol{\omega}} \mathcal{T} + \nabla_{\mathbf{v}} \mathcal{T} \times \mathbf{v} + \boldsymbol{\omega} \times \nabla_{\boldsymbol{\omega}} \mathcal{T}
   \end{align*}

ここで、 :math:`\mathcal{T}` は流体のみの運動エネルギーです。これらの力は、しばしば付加質量または仮想質量として説明されます。これは、加速するボディによって移動または偏向される流体の慣性によるものだからです。実際、一定の線速度を持つボディの場合、これらの力はゼロに減少します。この仮定の下で運動エネルギーが大幅に簡略化されるため、ボディは3つの対称面を持つと見なします。次のように書くことができます。

.. math::
   2 \mathcal{T} = m_{A, x} v_x^2 + m_{A, y} v_y^2 + m_{A, z} v_z^2 +
                 I_{A, x} \omega_x^2 + I_ {A, y} \omega_y^2 + I_{A, y} \omega_z^2


便宜上、付加質量ベクトル :math:`\mathbf{m}_A = \{m_{A, x}, m_{A, y}, m_{A, z}\}` と付加慣性モーメントベクトル :math:`\mathbf{I}_A = \{I_{A, x}, I_{A, y}, I_{A, z}\}` を導入します。これらの各量は、対応する方向へのボディの動きによって移動される流体の慣性を推定するものであり、いくつかの単純な形状についてポテンシャル流理論から導出できます。

3つの対称面を持つボディについて、付加慣性による力とトルクをコンパクトな形式で書くことができます。

.. math::
   \begin{align*}
       \mathbf{f}_{A} &= - \mathbf{m}_A \circ \dot{\mathbf{v}} + \left(\mathbf{m}_A \circ \mathbf{v} \right) \times \boldsymbol{\omega} \\
       \mathbf{g}_{A} &= - \mathbf{I}_A \circ \dot{\boldsymbol{\omega}} + \left(\mathbf{m}_A \circ \mathbf{v} \right) \times \mathbf{v} + \left(\mathbf{I}_A \circ \boldsymbol{\omega} \right) \times \boldsymbol{\omega}
   \end{align*}

ここで、 :math:`\circ` は要素ごとの積を表し、 :math:`\dot{\mathbf{v}}` は線加速度、
:math:`\dot{\boldsymbol{\omega}}` は角加速度です。 :math:`\mathbf{m}_A \circ \mathbf{v}` と
:math:`\mathbf{I}_A \circ \boldsymbol{\omega}` はそれぞれ仮想線運動量と仮想角運動量です。

半軸 :math:`\mathbf{r} = \{r_x, r_y, r_z\}` と体積 :math:`V = 4 \pi r_x r_y r_z / 3` の楕円体について、
仮想慣性係数は :cite:t:`tuckerman1925` によって導出されました。次のように定義します。

.. math::
   \kappa_i = \int_0^\infty \frac{r_i r_j r_k}{\sqrt{(r_i^2 + \lambda)^3 (r_j^2 + \lambda) (r_k^2 + \lambda)}} \textrm{d} \lambda


これらの係数は無次元であることに注意してください（つまり、すべての半軸が同じスカラーで乗算されても係数は変わりません）。楕円体の仮想質量は次のようになります。

.. math::
   m_{A, i} = \rho V \frac{\kappa_i}{2 - \kappa_i}

そして、仮想慣性モーメントは次のようになります。

.. math::
   I_{A, i} = \frac{\rho V}{5} \frac{(r_j^2 - r_k^2)^2 (\kappa_k-\kappa_j)}{2(r_j^2 - r_k^2) + (r_j^2 + r_k^2) (\kappa_j-\kappa_k)}

粘性抗力
~~~~~~~~~~~~

抗力は、周囲の流れに対するボディの運動を妨げるように作用します。粘性力は、流体力学項で拡張された運動方程式の剛性を低減するのにも役立つことがわかりました。このため、私たちは保守的な側に誤る方を選び、散逸を過大評価する可能性のある粘性項の近似を選択しました。

最終的に粘性散逸によって引き起こされるにもかかわらず、高レイノルズ数では、抗力は粘性とは無関係であり、速度の2乗に比例してスケールします。次のように書けます。

.. math::
   \begin{align*}
   \mathbf{f}_\text{D} = - C_D~\rho~ A_D ~ \|\mathbf{v}\|~ \mathbf{v}\\
   \mathbf{g}_\text{D} = - C_D \rho~ I_D ~ \|\boldsymbol{\omega}\| ~ \boldsymbol{\omega}
   \end{align*}

ここで、 :math:`C_D` は抗力係数、 :math:`A_D` は参照表面積（例えば、流れに垂直な平面上の投影面積の尺度）、 :math:`I_D` は参照慣性モーメントです。

.. youtube:: nljr0X79vI0
   :align: right
   :width: 50%

単純な形状でさえ、項 :math:`C_D`、 :math:`A_D`、 :math:`I_D` は問題固有の
物理と動的スケールに調整する必要があります :cite:p:`duan2015`。例えば、抗力係数 :math:`C_D` は一般にレイノルズ数の増加とともに減少し、単一の参照面積 :math:`A_D` では、非常に不規則または細長いボディの表面抗力を説明するのに十分でない場合があります。例えば、実験的フィットは、落下するトランプカード :cite:p:`wang2004,andersen2005a,andersen2005b` から粒子輸送 :cite:p:`loth2008,
bagheri2016` までの問題から導出されます。右側に
`cards.xml <https://github.com/deepmind/mujoco/blob/main/model/cards/cards.xml>`__ モデルのスクリーンキャプチャを参照してください。

2つの表面 :math:`A^\text{proj}_\mathbf{v}` と
:math:`A_\text{max}` に基づいた :math:`\mathbf{f}_\text{D}` の公式を導出します。最初の :math:`A^\text{proj}_\mathbf{v}` は、速度 :math:`\mathbf{v}` に垂直な平面上のボディの円筒投影です。2番目は最大投影表面
:math:`A_\text{max} = 4 \pi r_{max} r_{min}` です。

.. math::
   \mathbf{f}_\text{D} = - \rho~ \big[  C_{D, \text{blunt}} ~ A^\text{proj}_\mathbf{v} ~ +
   C_{D, \text{slender}}\left(A_\text{max} - A^\text{proj}_\mathbf{v} \right) \big] ~ \|\mathbf{v}\|~ \mathbf{v}

:math:`A^\text{proj}_\mathbf{v}` の公式と導出は、上記の :ref:`補題<flProjection>` で与えられています。

角度抗力について類似のモデルを提案します。各直交座標軸について、軸周りのボディの回転によって得られる最大掃引楕円体の慣性モーメントを考慮します。結果として得られる慣性モーメントの対角成分は次のようになります。

.. math::
   \mathbf{I}_{D,ii} = \frac{8\pi}{15} ~r_i ~\max(r_j, ~r_k)^4 .

この参照慣性モーメントを考慮すると、角度抗力トルクは次のように計算されます。

.. math::
   \mathbf{g}_\text{D} = - \rho ~ \boldsymbol{\omega} ~ \Big( \big[ C_{D, \text{angular}} ~ \mathbf{I}_D ~ +
   C_{D, \text{slender}} \left(\mathbf{I}_\text{max} - \mathbf{I}_D \right) \big] \cdot \boldsymbol{\omega} \Big)


ここで、 :math:`\mathbf{I}_\text{max}` は、 :math:`\mathbf{I}_D` の最大成分に等しい各エントリを持つベクトルです。

最後に、線形抗力としても知られる粘性抵抗項は、レイノルズ数が :math:`O(10)` 前後またはそれ以下の流体力をよく近似します。これらは、ストークスの法則 :cite:p:`stokes1850,lamb1932` を使用して等価球体について計算されます。

.. math::
   \begin{align*}
   \mathbf{f}_\text{V} &= - 6 \pi r_D \beta \mathbf{v}\\
   \mathbf{g}_\text{V} &= - 8 \pi r_D^3 \beta \boldsymbol{\omega}
   \end{align*}

ここで、 :math:`r_D = (r_x + r_y + r_z)/3` は等価球体の半径であり、 :math:`\beta` は媒体の動粘性係数です（例えば、室温の空気で :math:`1.48~\times 10^{-5}~m^2/s`、水で
:math:`0.89 \times 10^{-4}~m^2/s` です）。定量的な例を挙げると、ストークスの法則は、
:math:`u\cdot l \lesssim 2 \times 10^{-4}~m^2/s` の場合に室温の空気で正確になります。ここで、 :math:`u` は速度、
:math:`l` はボディの特性長さです。

粘性揚力
~~~~~~~~~~~~

クッタ・ジューコフスキーの定理は、速度 `u` で一様流を並進する2次元ボディの揚力 :math:`L` を :math:`L = \rho u \Gamma` として計算します。ここで、 :math:`\Gamma` はボディ周りの循環です。次のサブセクションでは、2つの循環源と結果として生じる揚力を定義します。

マグヌス力
^^^^^^^^^^^^

.. cssclass:: caption-small
.. figure:: ../images/computation/magnus.png
   :figwidth: 45%
   :align: right

   回転する円筒を通過する流れの煙流れの可視化（WikiMedia Commons、CC BY-SA 4.0）。粘性により、
   回転する円筒は入射流を上向きに偏向させ、下向きの力（赤い矢印）を受けます。

マグヌス効果は、流体中を移動する回転物体の運動を記述します。粘性効果を通じて、回転する物体は周囲の流体に回転を誘発します。この回転は、物体を通過する流体の軌道を偏向させます（つまり、線加速度を引き起こします）、そして物体は等しくかつ反対の反作用を受けます。円筒の場合、円筒の単位長さあたりのマグヌス力は :math:`F_\text{M} / L = \rho v \Gamma` として計算できます。ここで、 :math:`\Gamma` は回転によって引き起こされる流れの循環であり、 :math:`v` は物体の速度です。任意のボディについてこの力を次のように推定します。

.. math::
   \mathbf{f}_{\text{M}} = C_M ~\rho~ V~ \boldsymbol{\omega}\times\mathbf{v} ,

ここで、 :math:`V` はボディの体積であり、 :math:`C_M` は力の係数で、通常は1に設定されます。

例を挙げる価値があります。変数の数を減らすために、1方向のみに回転するボディ、例えば :math:`\boldsymbol{\omega} = \{0, 0, \omega_z\}` が他の2方向に沿って並進する、例えば :math:`\mathbf{v} = \{v_x, v_y, 0\}` と仮定します。例えば :math:`x` に沿った付加質量による力とマグヌス効果による力の和は次のようになります。

.. math::
   \frac{f}{\pi \rho r_z} = v_y \omega_z \left(2 r_x \min\{r_x, r_z\} - (r_x + r_z)^2\right)

2つの項が反対の符号を持つことに注意してください。

クッタ条件
^^^^^^^^^^^^^^^

よどみ点は、速度がゼロである流れ場内の位置です。流れ中を移動するボディ（2Dでは、ボディとともに移動するフレーム内）には2つのよどみ点があります。前方では、流線がボディの両側に分離する場所、後方では、それらが再接続する場所です。鋭い後縁（後方）エッジを持つ移動するボディは、周囲の流れ内に、後方のよどみ点を後縁に保持するのに十分な強さの循環を生成します。
これがクッタ条件であり、鋭いコーナーを持つ固体ボディ、例えば細長いボディや翼型の後縁で観察できる流体力学現象です。

.. figure:: ../images/computation/kutta_cond_plate.svg
   :class: only-light
   :figwidth: 95%
   :align: left

.. cssclass:: caption-small
.. figure:: ../images/computation/kutta_cond_plate_dark.svg
   :class: only-dark
   :figwidth: 95%
   :align: left

   クッタ条件のスケッチ。青い線は流線であり、2つのマゼンタ色の点はよどみ点です。2つのよどみ点を接続する分割流線は緑色でマークされています。分割流線とボディは、流れが「分離」され、その中で再循環すると言われる領域を囲みます。この循環は、プレートに作用する上向きの力を生み出します。

上の図にスケッチされた2次元流れの場合、クッタ条件による循環は次のように推定できます。
:math:`\Gamma_\text{K} = C_K ~ r_x ~ \| \mathbf{v}\| ~ \sin(2\alpha)`。
ここで、 :math:`C_K` は揚力係数であり、 :math:`\alpha` は速度ベクトルと表面への投影との間の角度です。単位長さあたりの揚力は、クッタ・ジューコフスキーの定理を使用して
:math:`\mathbf{f}_K / L = \rho \Gamma_\text{K} \times \mathbf{v}` として計算できます。

揚力方程式を3次元運動に拡張するために、法線
:math:`\mathbf{n}_{s, \mathbf{v}} = \{\frac{r_y r_z}{r_x}v_x, \frac{r_z r_x}{r_y}v_y, \frac{r_x r_x}{r_z}v_z\}`
を考慮します。これは、上記の :ref:`補題<flProjection>` で与えられた速度に垂直な平面上のボディの投影 :math:`A^\text{proj}_\mathbf{v}` を生成するボディの断面への法線であり、対応する単位ベクトル
:math:`\hat{\mathbf{n}}_{s, \mathbf{v}}` です。
この方向を使用して :math:`\mathbf{v} = \mathbf{v}_\parallel ~+~ \mathbf{v}_\perp` を分解します。ここで、
:math:`\mathbf{v}_\perp = \left(\mathbf{v} \cdot \hat{\mathbf{n}}_{s, \mathbf{v}}\right) \hat{\mathbf{n}}_{s, \mathbf{v}}` です。
揚力を次のように書きます。

.. math::
   \begin{align*}
       \mathbf{f}_\text{K} &= \frac{C_K~\rho~ A^\text{proj}_\mathbf{v}}{\|\mathbf{v}\|}
                                 \left( \mathbf{v} \times \mathbf{v}_\parallel\right)\times \mathbf{v} \\
       &= C_K~\rho~ A^\text{proj}_\mathbf{v} \left(\hat{\mathbf{v}} \cdot \hat{\mathbf{n}}_{s, \mathbf{v}}\right)
                                 \left( \hat{\mathbf{n}}_{s, \mathbf{v}} \times \mathbf{v} \right)\times \mathbf{v}
   \end{align*}

ここで、 :math:`\hat{\mathbf{v}}` は :math:`\mathbf{v}` に沿った単位法線です。 :math:`\hat{\mathbf{n}}_{s,
\mathbf{v}}` の方向は、ボディの半軸が等しくない平面上でのみ :math:`\hat{\mathbf{v}}` と異なることに注意してください。したがって、
例えば、球形ボディの場合、 :math:`\hat{\mathbf{n}}_{s, \mathbf{v}} \equiv \hat{\mathbf{v}}` であり、構成により
:math:`\mathbf{f}_\text{K} = 0` です。

関係を例で明らかにしましょう。 :math:`r_x = r_y` かつ :math:`r_z \ll r_x` のボディを仮定します。ベクトル
:math:`\hat{\mathbf{n}}_{s, \mathbf{v}} \times \hat{\mathbf{v}}` は、固体ボディによる流れの偏向によって誘発される循環の方向を与えることに注意してください。 :math:`z` に沿って、循環は :math:`\frac{r_y r_z}{r_x}v_x v_y
- \frac{r_z r_x}{r_y}v_x v_y = 0`（ :math:`r_x = r_y` のため）に比例します。したがって、固体が鈍い平面上では、運動は循環を生成しません。

次に、簡単のために :math:`v_x = 0` とします。この場合、 :math:`y` に沿った循環も、 :math:`\frac{r_y r_z}{r_x}v_x v_z - \frac{r_y r_x}{r_y}v_x v_z` に比例し、ゼロです。循環の唯一のゼロでない成分は :math:`x` に沿っており、 :math:`\left(\frac{r_x r_z}{r_y} - \frac{r_x r_y}{r_z}\right) v_y v_z \approx
\frac{r_x^2}{r_z} v_y v_z` に比例します。

:math:`\mathbf{v}_\parallel = \{v_x, 0, v_z\}` であり、
:math:`\Gamma \propto \{r_z v_y v_z, ~ 0,~ - r_x v_x v_y \} / \|\mathbf{v}\|` となります。
運動は、固体が鈍い平面上では循環を生成せず、他の2つの平面上では循環は
:math:`\Gamma \propto r_\Gamma ~ \|\mathbf{v}\|~ \sin(2 \alpha) ~ = ~2 r_\Gamma ~\|\mathbf{v}\| ~\sin(\alpha)~\cos(\alpha)`
です。ここで、 :math:`\alpha` は速度とその平面上のボディへの投影との間の角度です（例えば、 :math:`x` に垂直な平面上では :math:`\sin(\alpha) = v_y/\|\mathbf{v}\|` かつ
:math:`\cos(\alpha) = v_z/\|\mathbf{v}\|`）、 :math:`r_\Gamma` は平面上の揚力表面です（例えば、 :math:`x` に垂直な平面の場合は :math:`r_z` です）。さらに、循環の方向は外積によって与えられます（なぜなら、固体境界が入射流速をボディ上の投影に向かって「回転」させるからです）。

謝辞
~~~~~~~~~~~~~~~~

このセクションのモデルの設計と実装は、Guido Novatiの作品です。

参考文献
~~~~~~~~~~

.. bibliography::
