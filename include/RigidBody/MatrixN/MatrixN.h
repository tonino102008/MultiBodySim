#ifndef MULTIBODYSIM_INCLUDE_RIGIDBODY_MATRIXN_MATRIXN_H_
#define MULTIBODYSIM_INCLUDE_RIGIDBODY_MATRIXN_MATRIXN_H_

#define kSingleColumn 1
#define kSingleRow 1
#define kPi 3.14159265358979323846

#include <vector>
#include <ostream>

class MatrixN {

public:

	MatrixN(const int row_size, const int col_size);

	MatrixN(const int row_size, const int col_size, const std::vector<std::vector<double>> m);

	MatrixN(const int row_size, const int col_size, const double d);

	MatrixN(const int size, const double d);

	std::vector<int> getSize() const;

	std::vector<double>& operator[](const int i);

	std::vector<double> operator[](const int i) const;

	MatrixN& operator=(const MatrixN& m);

	MatrixN operator+(const MatrixN& m) const;

	MatrixN operator-(const MatrixN& m) const;

	MatrixN operator*(const MatrixN& m) const;

	MatrixN operator*(const double d) const;

	MatrixN operator/(const MatrixN& m) const;

	friend std::ostream& operator<<(std::ostream& out, const MatrixN& m);

	double dot(const MatrixN& m) const;

	MatrixN cross(const MatrixN& m) const;

	double norm() const;

	MatrixN T() const;

	void fill(const int fromRow, const int fromCol, MatrixN& m);

private:

	const int kRowSize_;

	const int kColSize_;

	std::vector<std::vector<double>> matrix_;

};

#endif // MULTIBODYSIM_INCLUDE_RIGIDBODY_MATRIXN_MATRIXN_H_