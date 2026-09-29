"""Shared Sphinx configuration; sources are selected by DOCS_LANG."""

import os
import sys
from pathlib import Path

DOCS = Path(__file__).resolve().parent
ROOT = DOCS.parent
sys.path.insert(0, str(DOCS / 'scripts'))
from metadata import load_metadata, substitute  # noqa: E402

metadata = load_metadata()
project = metadata['name']
author = project
version = release = metadata['version']
language = os.environ.get('DOCS_LANG', 'ru')
print_build = os.environ.get('DOCS_PRINT') == '1'
extensions = ['myst_parser', 'breathe']
source_suffix = {'.md': 'markdown'}
root_doc = 'manual' if print_build else 'index'
exclude_patterns = ['manual.md'] if not print_build else ['index.md', 'reference/cpp-api.md']
myst_enable_extensions = ['colon_fence']
myst_heading_anchors = 3
html_theme = 'sphinx_rtd_theme'
html_theme_options = {'navigation_depth': 3, 'collapse_navigation': False}
html_static_path = [str(DOCS / 'assets')]
html_css_files = ['styles/site.css']
html_title = project + ' — техническое руководство'
html_show_sourcelink = False
html_copy_source = False
html_show_sphinx = False
html_show_copyright = False
html_search_language = language
breathe_projects = {'metro': str(ROOT / 'results/docs' / language / 'doxygen/xml')}
breathe_default_project = 'metro'
breathe_domain_by_extension = {'hpp': 'cpp'}


def relative_assets(app, pagename, templatename, context, doctree):
    """Keep authored raw illustrations usable when the site lives in a subdirectory."""
    prefix = '../' * pagename.count('/') + '_static/'
    context['body'] = context.get('body', '').replace('/_static/', prefix)


def setup(app):
    app.connect('html-page-context', relative_assets)
    app.connect('source-read', publication_metadata)


def publication_metadata(app, docname, source):
    source[0] = substitute(source[0], metadata)
