# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import numpy as np

from kiss_icp.config import KISSConfig
from kiss_icp.pybind import kiss_icp_pybind


def get_preprocessor(config: KISSConfig):
    return Preprocessor(
        max_range=config.data.max_range,
        min_range=config.data.min_range,
        deskew=config.data.deskew,
        max_num_threads=config.registration.max_num_threads,
    )


class Preprocessor:
    def __init__(self, max_range, min_range, deskew, max_num_threads):
        self._preprocessor = kiss_icp_pybind._Preprocessor(
            max_range, min_range, deskew, max_num_threads
        )

    def preprocess(self, frame: np.ndarray, timestamps: np.ndarray, relative_motion: np.ndarray):
        return np.asarray(
            self._preprocessor._preprocess(
                kiss_icp_pybind._Vector3dVector(frame),
                timestamps.ravel(),
                relative_motion,
            )
        )
