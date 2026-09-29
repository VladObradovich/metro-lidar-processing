"""Resolve shared publication metadata for Sphinx, the cover and print margins."""

import html
import os
from datetime import date, datetime, timezone
from pathlib import Path
from xml.etree import ElementTree

import yaml

DOCS = Path(__file__).resolve().parents[1]
ROOT = DOCS.parent


def load_metadata():
    metadata = yaml.safe_load((DOCS / 'project.yaml').read_text())
    package = ElementTree.parse(ROOT / metadata['version_source']).getroot()
    version = package.findtext('version', '').strip()
    if not version:
        raise ValueError('Missing project version in ' + metadata['version_source'])
    explicit_date = os.environ.get('DOCS_DATE', '').strip()
    epoch = os.environ.get('SOURCE_DATE_EPOCH', '').strip()
    if explicit_date:
        publication_date = date.fromisoformat(explicit_date)
    elif epoch:
        publication_date = datetime.fromtimestamp(int(epoch), timezone.utc).date()
    else:
        publication_date = date.today()
    metadata.update(
        version=version,
        date=publication_date.isoformat(),
        date_display=publication_date.strftime('%d.%m.%Y'),
    )
    return metadata


def substitute(text, metadata):
    """Expand only publication placeholders, including those inside raw HTML/SVG/CSS."""
    values = {'doc_version': 'v' + metadata['version'], 'doc_date': metadata['date_display']}
    for key, value in values.items():
        text = text.replace('{{ ' + key + ' }}', html.escape(value, quote=True))
    return text
