..
   このファイルは index.html としては使用されず、toctree の定義のみに使われます。
   これは、toctree が定義されているページ自身への参照を含むことができないためです
   (https://github.com/sphinx-doc/sphinx/issues/4602)。
   conf.py の `redirects` 設定により、このページは overview.html にリダイレクトされます。

.. toctree::
   :hidden:

   overview
   computation/index.rst
   modeling
   XMLreference
   programming/index.rst
   APIreference/index.rst
   python
   MuJoCo XLA <mjx>
   MuJoCo Warp <mjwarp/index>
   OpenUSD <OpenUSD/index>
   unity
   models
   changelog


.. sidebar-links::
   :github:
