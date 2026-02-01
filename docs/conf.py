# MuJoCo 日本語ドキュメント - Sphinx設定
# オリジナル: https://github.com/google-deepmind/mujoco

import os
import sys

from sphinxcontrib import katex

# -- パス設定 ----------------------------------------------------------------

sys.path.append(os.path.abspath("ext"))

# -- プロジェクト情報 ---------------------------------------------------------

project = "MuJoCo"
copyright = "DeepMind Technologies Limited"
author = "Google DeepMind"
language = "ja"

# -- 一般設定 ----------------------------------------------------------------

master_doc = "index"

# Sphinx拡張
extensions = [
    "sphinx.ext.extlinks",
    "sphinx.ext.napoleon",
    "sphinxcontrib.bibtex",
    "sphinxcontrib.katex",
    "sphinxcontrib.youtube",
    "sphinx_copybutton",
    "sphinx_design",
    "sphinx_favicon",
    "sphinx_reredirects",
    "sphinx_toolbox.collapse",
    "sphinx_toolbox.github",
    "sphinx_toolbox.sidebar_links",
    "mujoco_include",
]

# Napoleon設定（MuJoCo Warp用）
napoleon_custom_sections = [("warp only fields", "attributes")]

# GitHubリンク用
extlinks = {
    "issue": (
        "https://github.com/google-deepmind/mujoco/issues/%s",
        "issue #%s",
    ),
    "pr": ("https://github.com/google-deepmind/mujoco/pull/%s", "PR #%s"),
}

# MuJoCo Warp ドキュメント用設定
napoleon_google_docstring = True

# GitHub関連設定
github_username = "google-deepmind"
github_repository = "mujoco"

# Bibtex参考文献設定
bibtex_bibfiles = ["references.bib"]

# テンプレートパス
templates_path = ["templates"]

# 除外パターン
exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "includes/*",
    "APIreference/functions.rst",
    "APIreference/functions_override.rst",
    "XMLschema.rst",
]

# リダイレクト設定
# index.rstは目次定義のみを含むため、overview.htmlにリダイレクト
redirects = {
    "index": "overview.html",
    "computation": "computation/index.html",
    "programming": "programming/index.html",
    "APIreference": "APIreference/index.html",
}

# RSTプロローグ（共通のインクルード）
rst_prolog = """
.. include:: /includes/macros.rst
.. include:: /includes/roles.rst
.. include:: <isonum.txt>
"""

# -- HTML出力オプション -------------------------------------------------------

html_theme = "furo"
html_title = "MuJoCo ドキュメント 日本語版"
html_logo = "images/banner.svg"

SHARED_CSS_VARIABLES = {
    "admonition-font-size": "1rem",
    "admonition-title-font-size": "1rem",
    "sidebar-item-font-size": "130%",
}

# font-stack--monospace: コードブロックで使用、Inconsolataは100文字に最適化
html_theme_options = {
    "light_css_variables": {
        "font-stack--monospace": "Inconsolata,Consolas,ui-monospace,monospace",
        "at-color": "#830b2b",
        "at-val-color": "#bc103e",
        "body-color": "#14234b",
        "color-highlight-on-target": "#e5e8ed",
        "primary-header-color": "#0053d6",
        "row-odd-background-color": "#f0f3f7",
        "rst-content-a-color": "#2980b9",
        "secondary-header-color": "#123693",
        "wy-menu-vertical-background-color": "#0053d6",
        "wy-menu-vertical-color": "white",
        "wy-nav-side-background-color": "#0053d6",
    },
    "dark_css_variables": {
        "at-color": "#ffaab7",
        "at-val-color": "#ff95a6",
        "body-color": "#14234b",
        "color-admonition-background": "#1e1e21",
        "color-highlight-on-target": "#3d4045",
        "primary-header-color": "#a8caff",
        "row-odd-background-color": "#222326",
        "rst-content-a-color": "#2980b9",
        "secondary-header-color": "#458dff",
        "wy-menu-vertical-background-color": "#0053d6",
        "wy-menu-vertical-color": "white",
        "wy-nav-side-background-color": "#0053d6",
    },
}

for v in html_theme_options.values():
    if isinstance(v, dict):
        v.update(SHARED_CSS_VARIABLES)

pygments_style = "default"
pygments_dark_style = "monokai"

# カスタム静的ファイル（スタイルシートなど）のパス
# ビルトインの静的ファイルの後にコピーされるため、同名ファイルで上書き可能
html_static_path = [
    "_static",
    "css",
    "js",
]
html_css_files = [
    "theme_overrides.css",
    "theme_overrides_mjwarp.css",
]
html_js_files = [
    "linenumbers.js",
    "onthispage_mjwarp.js",
]

favicons = [
    {
        "sizes": "16x16",
        "href": "favicons/favicon-16x16.png",
    },
    {
        "sizes": "32x32",
        "href": "favicons/favicon-32x32.png",
    },
    {
        "rel": "apple-touch-icon",
        "sizes": "180x180",
        "href": "favicons/favicon-180x180.png",
    },
    {
        "sizes": "180x180",
        "href": "favicons/favicon-180x180.png",
    },
    {
        "sizes": "192x192",
        "href": "favicons/favicon-192x192.png",
    },
]

html_permalinks_icon = "#"

# -- KaTeX数式設定 ------------------------------------------------------------

# 参照: https://sphinxcontrib-katex.readthedocs.io/en/0.4.1/macros.html
# {ar, au, ac} はそれぞれ {reference, unconstrained, constrained} の加速度
latex_macros = r"""
    \def \d              #1{\operatorname{#1}}
    \def \ar             {a_{\rm ref}}
    \def \au             {a_0}
    \def \ac             {a_1}
    \def \ari            {a_{{\rm ref},i}}
    \def \aui            {a_{0,i}}
    \def \aci            {a_{1,i}}
    \def \nv             {n_{\scriptscriptstyle V}}
    \def \nc             {n_{\scriptscriptstyle C}}
    \def \nq             {n_{\scriptscriptstyle Q}}
"""

# LaTeXマクロをKaTeX形式に変換してHTMLビルダーのオプションに追加
katex_macros = katex.latex_defs_to_katex_macros(latex_macros)
katex_options = "macros: {" + katex_macros + "}"

# LaTeXビルダー用にマクロを追加
latex_elements = {"preamble": latex_macros}
