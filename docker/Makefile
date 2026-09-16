SHELL := /bin/bash
.ONESHELL:
.SHELLFLAGS := -eu -o pipefail -c
.DELETE_ON_ERROR:
MAKEFLAGS += --warn-undefined-variables
MAKEFLAGS += --no-builtin-rules

.DEFAULT_GOAL := help

.PHONY: up down logs yamcs-shell grafana-shell simulator-shell image-shell wait-for-yamcs help

up: ## start Yamcs, the simulator, the image generator, and Grafana
	docker compose -f compose.yaml up --build

down: ## stop the demo stack
	docker compose -f compose.yaml down -v --remove-orphans

logs: ## follow all demo logs
	docker compose -f compose.yaml logs -f

yamcs-shell: ## open a shell in the Yamcs container
	docker compose -f compose.yaml exec yamcs bash

grafana-shell: ## open a shell in the Grafana container
	docker compose -f compose.yaml exec grafana bash

simulator-shell: ## open a shell in the simulator container
	docker compose -f compose.yaml exec simulator bash

image-shell: ## open a shell in the image generator container
	docker compose -f compose.yaml exec image-generator bash

wait-for-yamcs: ## CI helper: wait until the Yamcs HTTP service is reachable
	docker compose -f compose.yaml up -d --build yamcs simulator
	timeout=600
	until docker compose -f compose.yaml exec -T yamcs curl -fsS http://localhost:8090 >/dev/null; do
		sleep 1
		timeout=$$((timeout - 1))
		if [ "$$timeout" -le 0 ]; then
			docker compose -f compose.yaml logs
			exit 1
		fi
	done

help: ## display this help message
	@awk 'BEGIN {FS = ":.*##"; printf "\033[36m%-20s\033[0m %s\n", "Target", "Description"} /^[a-zA-Z_-]+:.*?##/ {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST) | sort
