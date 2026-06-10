# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Paths for the native ``crp_getui_probe`` binary (lives next to this file)."""

from __future__ import annotations

from pathlib import Path

GETUI_PROBE_DIR = Path(__file__).resolve().parent
DEFAULT_GETUI_PROBE_BINARY = GETUI_PROBE_DIR / "crp_getui_probe"
BUILD_SCRIPT = GETUI_PROBE_DIR / "build.sh"
