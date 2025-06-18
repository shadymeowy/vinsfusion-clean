from dataclasses import dataclass
from typing import Dict
import numpy as np
import numba as nb


@dataclass
class TrackSet:
    points: np.ndarray
    validity: np.ndarray = None
    lifetime: np.ndarray = None
    ids: np.ndarray = None
    desc: np.ndarray = None

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float)

        if not (self.points.ndim == 2 and self.points.shape[1] == 2):
            raise ValueError("Invalid points array")

        n = self.points.shape[0]

        if self.validity is not None:
            self.validity = np.asarray(self.validity, dtype=bool)
        else:
            self.validity = np.ones((n,), dtype=bool)

        if self.lifetime is not None:
            self.lifetime = np.asarray(self.lifetime, dtype=float)
        else:
            self.lifetime = np.zeros((n,), dtype=float)

        if self.ids is not None:
            self.ids = np.asarray(self.ids, dtype=int)
        else:
            self.ids = None

        if self.desc is not None:
            self.desc = np.asarray(self.desc, dtype=float)
        else:
            self.desc = None

    def __getitem__(self, idx: slice | int | np.ndarray):
        return TrackSet(
            points=self.points[idx],
            validity=self.validity[idx],
            lifetime=self.lifetime[idx],
            ids=self.ids[idx] if self.ids is not None else None,
            desc=self.desc[idx] if self.desc is not None else None,
        )

    def __setitem__(self, idx: slice | int | np.ndarray, value: "TrackSet"):
        if not isinstance(value, TrackSet):
            raise ValueError("Value must be a TrackSet")

        self.points[idx] = value.points
        self.validity[idx] = value.validity
        self.lifetime[idx] = value.lifetime
        if self.ids is not None:
            self.ids[idx] = value.ids
        if self.desc is not None:
            self.desc[idx] = value.desc

    @property
    def x(self):
        return self.points[:, 0]

    @property
    def y(self):
        return self.points[:, 1]

    def __len__(self):
        return len(self.points)

    def as_dict(self):
        return {
            "points": self.points,
            "validity": self.validity,
            "ids": self.ids,
            "lifetime": self.lifetime,
            "desc": self.desc,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, np.ndarray]):
        return cls(
            points=data["points"],
            validity=data["validity"],
            ids=data["ids"],
            lifetime=data["lifetime"],
            desc=data["desc"],
        )

    @classmethod
    def empty(cls):
        return cls(
            points=np.empty((0, 3)),
            valid=np.empty((0,), dtype=bool),
            ids=np.empty((0,), dtype=int),
            lifetime=np.empty((0,), dtype=float),
            desc=None,
        )

    def __add__(self, other):
        if not isinstance(other, TrackSet):
            raise ValueError("Can only add TrackSet to TrackSet")

        return TrackSet(
            points=np.vstack((self.points, other.points)),
            validity=np.hstack((self.validity, other.validity)),
            ids=np.hstack((self.ids, other.ids))
            if self.ids is not None and other.ids is not None
            else None,
            lifetime=np.hstack((self.lifetime, other.lifetime)),
            desc=np.vstack((self.desc, other.desc))
            if self.desc is not None and other.desc is not None
            else None,
        )

    def valid(self):
        return TrackSet(
            points=self.points[self.validity],
            validity=self.validity[self.validity],
            ids=self.ids[self.validity] if self.ids is not None else None,
            lifetime=self.lifetime[self.validity],
            desc=self.desc[self.validity] if self.desc is not None else None,
        )


if __name__ == "__main__":
    t1 = TrackSet([[0, 0], [1, 2], [2, 3]])
    print(t1)

    t2 = TrackSet([[3, 4], [5, 6], [7, 8]])

    t1[:1] = t2[:1]
    print(t1)

    t1.x[:] = 0
    print(t1)

    t1.y[[False, True, False]] = 1
    print(t1)
