SPHINXBUILD ?= sphinx-build
SPHINXOPTS ?= -W --keep-going

.PHONY: html linkcheck serve clean
html:
	$(SPHINXBUILD) -b dirhtml $(SPHINXOPTS) . _build/html
linkcheck:
	$(SPHINXBUILD) -b linkcheck $(SPHINXOPTS) . _build/linkcheck
serve:
	adoc serve -d .
clean:
	rm -rf _build
