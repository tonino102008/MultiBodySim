#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODYCONST_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODYCONST_H_

#include "MatrixN/MatrixN.h"

const MatrixN kDGDqs(3, 4, { {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0} });
const MatrixN kDGDqx(3, 4, { {-1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, -1.0, 0.0} });
const MatrixN kDGDqy(3, 4, { {0.0, 0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0} });
const MatrixN kDGDqz(3, 4, { {0.0, 0.0, 1.0, 0.0}, {0.0, -1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0, 0.0} });

const double kAlpha = 1000.0;
const double kBeta = 1000.0;

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_RIGIDBODYCONST_H_