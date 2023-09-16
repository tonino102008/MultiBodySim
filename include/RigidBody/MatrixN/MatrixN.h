#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_MATRIXN_MATRIXN_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_MATRIXN_MATRIXN_H_

#include <vector>
#include <ostream>

class MatrixN {

public:

	MatrixN(const int row_size, const int col_size);

	std::vector<int> getSize() const;

	std::vector<double>& operator[](const int i);

	std::vector<double> operator[](const int i) const;

	MatrixN& operator=(const MatrixN& v);

	MatrixN operator+(const MatrixN& v) const;

	MatrixN operator-(const MatrixN& v) const;

	MatrixN operator*(const MatrixN& v) const;

	MatrixN operator*(const double d) const;

	friend std::ostream& operator<<(std::ostream& out, const MatrixN& m);

private:

	const int kRowSize_;

	const int kColSize_;

	std::vector<std::vector<double>> matrix_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_MATRIXN_MATRIXN_H_