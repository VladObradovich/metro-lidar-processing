"""Render the Sphinx single-page manual with the reviewed print design."""

import html
import re
from pathlib import Path

from bs4 import BeautifulSoup
from metadata import load_metadata, substitute
from weasyprint import HTML

DOCS = Path(__file__).resolve().parents[1]


def render(output, language, metadata=None):
    metadata = metadata or load_metadata()
    soup = BeautifulSoup((output / 'print/manual.html').read_text(), 'html.parser')
    for node in soup.select('.headerlink'):
        node.decompose()
    parts = []
    toc = (
        '<section class="toc"><div class="part-title">НАВИГАЦИЯ ПО РУКОВОДСТВУ</div>'
        '<h1>Содержание</h1>'
    )
    previous = None
    for chapter in metadata['manual']:
        marker = soup.find(id='document-' + chapter['path'])
        if marker is None:
            raise ValueError('Missing chapter in Sphinx output: ' + chapter['path'])
        container = marker.find_next('section')
        heading = container.find(re.compile(r'^h[1-6]$'))
        heading.name = 'h1'
        heading['id'] = chapter['id']
        heading.clear()
        heading.append(
            BeautifulSoup(
                '<span>' + chapter['number'] + '</span> ' + html.escape(chapter['title']),
                'html.parser',
            )
        )
        labels = re.findall(
            r'^\((s-[^)]+)\)=$', (DOCS / language / (chapter['path'] + '.md')).read_text(), re.M
        )
        subheadings = [
            node
            for node in container.find_all(re.compile(r'^h[2-6]$'))
            if not node.get('id', '').startswith('s-')
        ]
        if len(labels) != len(subheadings):
            raise ValueError('Subheading labels do not match: ' + chapter['path'])
        for node, label in zip(subheadings, labels):
            node.name = 'h2'
            node['id'] = label
        for wrapper in list(container.find_all('section')):
            wrapper.unwrap()
        # Avoid repeated target IDs from Sphinx wrappers after heading restoration.
        for node in container.find_all('span', id=re.compile(r'^chapter-|^s-')):
            node.unwrap()
        for node in container.find_all('span', id=re.compile(r'^id\d+$')):
            if not node.get_text():
                node.decompose()
        # Sphinx separates raw blocks with newlines; keep the reviewed compact flow.
        for node in list(container.children):
            if isinstance(node, str) and not node.strip():
                node.extract()
        for image in container.find_all('img'):
            source = image.get('src', '')
            if '_static/' in source:
                image['src'] = (DOCS / 'assets' / source.split('_static/', 1)[1]).as_uri()
        for link in container.find_all('a', href=True):
            href = link['href']
            if '#' in href and not href.startswith(('https:', 'http:')):
                link['href'] = '#' + href.rsplit('#', 1)[1]
        parts.append(
            '<section class="chapter"><div class="part">'
            + chapter['part']
            + '</div>'
            + container.decode_contents()
            + '</section>'
        )
        if chapter['part'] != previous:
            toc += '<div class="part-title">' + chapter['part'] + '</div>'
            previous = chapter['part']
        toc += (
            '<a class="toc-row" href="#'
            + chapter['id']
            + '"><b>'
            + chapter['number']
            + '</b> &nbsp; '
            + chapter['title']
            + '</a>'
        )
    toc += (
        '<div class="colophon">Заголовки оглавления кликабельны. '
        'Подразделы доступны в закладках PDF. Каждый блок команд сопровождается '
        'контекстом запуска; значения справочников извлечены из снимка исходников.'
        '</div></section>'
    )
    cover = (
        '<section class="cover">'
        + substitute((DOCS / 'assets/cover/title.svg').read_text(), metadata)
        + '</section>'
    )
    css = substitute((DOCS / 'assets/styles/print.css').read_text(), metadata)
    css += '\nfigure img{display:block;width:100%;height:auto}\n'
    document = (
        '<!doctype html><html lang="' + language + '"><head><meta charset="utf-8">'
        '<title>Metro Perception — техническое руководство</title>'
        '<meta name="author" content="Metro Perception"><style>'
        + css
        + '</style></head><body>'
        + cover
        + toc
        + ''.join(parts)
        + '</body></html>'
    )
    destination = output / 'metro-perception-manual.html'
    destination.write_text(document)
    rendered = HTML(filename=str(destination)).render()
    rendered.write_pdf(str(output / 'metro-perception-manual.pdf'), pdf_tags=True)
    return {
        'html': str(output / 'html/index.html'),
        'pdf': str(output / 'metro-perception-manual.pdf'),
        'pages': len(rendered.pages),
        'chapters': len(parts),
        'subsections': len(soup.select('h2[id^="s-"]')),
        'tables': document.count('<table>'),
        'figures': document.count('<figure>'),
    }
