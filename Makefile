DOCS_LANG ?= ru
DOCS_HOST ?= 127.0.0.1
DOCS_PORT ?= 8000
DOCS_PYTHON ?= .venv-docs/bin/python
DOCS_DATE ?=
export DOCS_DATE

.PHONY: docs-venv docs-host docs-build-host docs-help

docs-venv:
	python3 -m venv .venv-docs
	.venv-docs/bin/python -m pip install -r docs/requirements.txt

docs-host:
	@test -x "$(DOCS_PYTHON)" || { echo "Run make docs-venv first"; exit 1; }
	$(DOCS_PYTHON) docs/scripts/build.py serve --language "$(DOCS_LANG)" --host "$(DOCS_HOST)" --port "$(DOCS_PORT)"

docs-build-host:
	@test -x "$(DOCS_PYTHON)" || { echo "Run make docs-venv first"; exit 1; }
	$(DOCS_PYTHON) docs/scripts/build.py build --language "$(DOCS_LANG)"

docs-help:
	@echo 'make docs-venv       Install Python dependencies into .venv-docs'
	@echo 'make docs-host       Live site at http://127.0.0.1:8000 (Ctrl+C to stop)'
	@echo 'make docs-build-host Build HTML, C++ API and PDF into results/docs/ru'
	@echo 'Overrides: DOCS_LANG, DOCS_HOST, DOCS_PORT, DOCS_PYTHON'
	@echo 'Date defaults to build day; set DOCS_DATE=YYYY-MM-DD or SOURCE_DATE_EPOCH to pin it'
	@echo 'System dependency: doxygen; WeasyPrint also requires Pango and fonts'
