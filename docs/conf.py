# MuJoCo 日本語ドキュメント - Sphinx設定
# オリジナル: https://github.com/google-deepmind/mujoco

import os
import sys

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
    "sphinx_copybutton",
    "sphinx_design",
]

# GitHubリンク用
extlinks = {
    "issue": (
        "https://github.com/google-deepmind/mujoco/issues/%s",
        "issue #%s",
    ),
    "pr": ("https://github.com/google-deepmind/mujoco/pull/%s", "PR #%s"),
}

# テンプレートパス
templates_path = ["templates"]

# 除外パターン
exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "includes/*",
]

# RSTプロローグ（共通のインクルード）
rst_prolog = """
.. include:: <isonum.txt>
"""

# -- HTML出力オプション -------------------------------------------------------

html_theme = "furo"
html_title = "MuJoCo ドキュメント 日本語版"
html_static_path = [
    "_static",
    "css",
    "js",
]

# Furoテーマオプション
SHARED_CSS_VARIABLES = {
    "admonition-font-size": "1rem",
    "admonition-title-font-size": "1rem",
    "sidebar-item-font-size": "130%",
}

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
    },
}

for v in html_theme_options.values():
    if isinstance(v, dict):
        v.update(SHARED_CSS_VARIABLES)

pygments_style = "default"
pygments_dark_style = "monokai"
html_permalinks_icon = "#"
