.PHONY: refresh all

all : cJSON_test.cc
	@echo Number of generated tests:
	@grep 'TEST' $< | wc -l

refresh :
	@echo refresh??
	@touch SpewJson.py

cJSON_test.cc: Traces3.py Traces10.py Traces20.py Traces50.py Traces100.py TracesLeadingZeroes.py TracesMultipleRoots.py Traces2000.py \
				TracesValidObjects.py
	@echo Create $@
	@python CreateGtest.py $(basename $^) >$@

Traces3.py : MaxLength3.py SpewJson.py
	@echo Generate 20 3 characters long objects
	@pmt -r 20 -o $(basename $@) $(basename $^) >/dev/null

Traces10.py : MaxLength10.py SpewJson.py
	@echo Generate 50 10 characters long objects
	@pmt -r 50 -o $(basename $@) $(basename $^) >/dev/null

Traces20.py : MaxLength20.py SpewJson.py
	@echo Generate 50 20 characters long objects
	@pmt -r 50 -o $(basename $@) $(basename $^) >/dev/null

Traces50.py : MaxLength50.py SpewJson.py
	@echo Generate 150 50 characters long objects
	@pmt -r 150 -o $(basename $@) $(basename $^) >/dev/null

Traces100.py : MaxLength100.py SpewJson.py
	@echo Generate 300 100 characters long objects
	@pmt -r 300 -o $(basename $@) $(basename $^) >/dev/null

Traces2000.py : MaxLength2000.py SpewJson.py
	@echo Generate 2000 characters long object
	@pmt -o $(basename $@) $(basename $^) >/dev/null

TracesLeadingZeroes.py : AllowLeadingZeroes.py SpewJson.py
	@echo Generate 300 objects, where integers with leading zeroes are allowed
	@pmt -r 300 -o $(basename $@) SpewJson AllowLeadingZeroes >/dev/null

TracesMultipleRoots.py : AllowMultipleRoots.py SpewJson.py
	@echo Generate 5 objects, where multiple root objects exist
	@pmt -r 5 -o $(basename $@) SpewJson AllowMultipleRoots >/dev/null

TracesValidObjects.py : MostlyValid.py SpewJson.py
	@echo Generate 500 objects, most of which should be correct JSON
	@printf '    Number of valid objects: '
	@pmt -r 500 -o $(basename $@) SpewJson MostlyValid | grep reached | wc -l

