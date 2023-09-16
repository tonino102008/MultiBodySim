#include "RigidBody/MatrixN/MatrixN.h"

MatrixN::MatrixN(const int row_size, const int col_size) :
	kRowSize_(row_size), 
	kColSize_(col_size),
	matrix_(kRowSize_, std::vector<double>(kColSize_))
{};

std::vector<int> MatrixN::getSize() const {
	return std::vector<int> {kRowSize_, kColSize_};
};

std::vector<double>& MatrixN::operator[](const int i) {
	return this->matrix_[i];
};

std::vector<double> MatrixN::operator[](const int i) const {
	return this->matrix_[i];
};

std::ostream& operator<<(std::ostream& out, const MatrixN& m) { // For now only 2D Matrices
	for (int i = 0; i < m.getSize()[0]; i++)
	{
		for (int j = 0; j < m.getSize()[1]; j++) {
			out << m[i][j] << " ";
		}
		if (i != (m.getSize()[0] - 1))
			out << "\n";
	}
	return out;
};