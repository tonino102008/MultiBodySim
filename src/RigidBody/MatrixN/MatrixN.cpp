#include "RigidBody/MatrixN/MatrixN.h"

MatrixN::MatrixN(const int row_size, const int col_size) :
	kRowSize_(row_size),
	kColSize_(col_size),
	matrix_(kRowSize_, std::vector<double>(kColSize_))
{};

MatrixN::MatrixN(const int row_size, const int col_size, const std::vector<std::vector<double>> m) :
	kRowSize_(row_size),
	kColSize_(col_size),
	matrix_(m)
{};

MatrixN::MatrixN(const int row_size, const int col_size, const double d) :
	kRowSize_(row_size),
	kColSize_(col_size),
	matrix_(kRowSize_, std::vector<double>(kColSize_))
{
	for (int i = 0; i < this->getSize()[0]; i++)
		for (int j = 0; j < this->getSize()[1]; j++)
			this->matrix_[i][j] = d;
};

MatrixN::MatrixN(const int size, const double d) :
	kRowSize_(size),
	kColSize_(size),
	matrix_(kRowSize_, std::vector<double>(kColSize_))
{
	for (int i = 0; i < this->getSize()[0]; i++)
		this->matrix_[i][i] = d;
};

std::vector<int> MatrixN::getSize() const {
	return std::vector<int> {kRowSize_, kColSize_};
};

std::vector<double>& MatrixN::operator[](const int i) {
	return this->matrix_[i];
};

std::vector<double> MatrixN::operator[](const int i) const {
	return this->matrix_[i];
};

MatrixN& MatrixN::operator=(const MatrixN& m) {
	this->matrix_ = m.matrix_;
	return *this;
};

MatrixN MatrixN::operator+(const MatrixN& m) const {
	MatrixN out(this->getSize()[0], this->getSize()[1]);
	for (int i = 0; i < this->getSize()[0]; i++)
		for (int j = 0; j < this->getSize()[1]; j++)
			out[i][j] = this->matrix_[i][j] + m[i][j];
	return out;
};

MatrixN MatrixN::operator-(const MatrixN& m) const {
	MatrixN out(this->getSize()[0], this->getSize()[1]);
	for (int i = 0; i < this->getSize()[0]; i++)
		for (int j = 0; j < this->getSize()[1]; j++)
			out[i][j] = this->matrix_[i][j] - m[i][j];
	return out;
};

MatrixN MatrixN::operator*(const MatrixN& m) const {
	MatrixN out(this->getSize()[0], m.getSize()[1]);
	for (int i = 0; i < this->getSize()[0]; i++)
		for (int j = 0; j < m.getSize()[1]; j++)
			for (int k = 0; k < m.getSize()[0]; k++)
				out[i][j] += this->matrix_[i][k] * m[k][j];
	return out;
};

MatrixN MatrixN::operator*(const double d) const {
	MatrixN out(this->getSize()[0], this->getSize()[1]);
	for (int i = 0; i < this->getSize()[0]; i++)
		for (int j = 0; j < this->getSize()[1]; j++)
			out[i][j] = this->matrix_[i][j] * d;
	return out;
};

MatrixN MatrixN::operator/(const MatrixN& m) const {
	MatrixN U(this->getSize()[0], this->getSize()[1]);
	MatrixN L(this->getSize()[0], 1.0);
	MatrixN out(this->getSize()[0], kSingleColumn);
	double sum;
	int N = this->getSize()[0];

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (j < i)
				L[j][i] = 0;
			else {
				L[j][i] = this->matrix_[j][i];
				for (int k = 0; k < i; k++) {
					L[j][i] = L[j][i] - L[j][k] * U[k][i];
				}
			}
		}
		for (int j = 0; j < N; j++) {
			if (j < i)
				U[i][j] = 0;
			else if (j == i)
				U[i][j] = 1;
			else {
				U[i][j] = this->matrix_[i][j] / L[i][i];
				for (int k = 0; k < i; k++) {
					U[i][j] = U[i][j] - ((L[i][k] * U[k][j]) / L[i][i]);
				}
			}
		}
	}

	for (int i = 0; i < N; i++) {
		sum = 0;
		for (int j = 0; j < i; j++)
			sum += L[i][j] * out[j][0];
		out[i][0] = (m[i][0] - sum) / L[i][i];
	}

	for (int i = (N - 1); i >= 0; i--) {
		sum = 0;
		for (int j = (N - 1); j > i; j--)
			sum += U[i][j] * out[j][0];
		out[i][0] = (out[i][0] - sum) / U[i][i];
	}

	return out;
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

double MatrixN::dot(const MatrixN& m) const {
	if ((this->getSize()[1] != kSingleColumn) || (m.getSize()[1] != kSingleColumn)) {
		double out = 0;
		return out;
	}
	double out = 0;
	for (int i = 0; i < this->getSize()[0]; i++)
		out += this->matrix_[i][0] * m[i][0];
	return out;
};

MatrixN MatrixN::cross(const MatrixN& m) const {
	if ((this->getSize()[0] != 3) || (m.getSize()[0] != 3) || (this->getSize()[1] != kSingleColumn) || (m.getSize()[1] != kSingleColumn)) {
		MatrixN out(0,0);
		return out;
	}
	MatrixN out(this->getSize()[0],kSingleColumn);
	out[0][0] = this->matrix_[1][0] * m[2][0] - this->matrix_[2][0] * m[1][0];
	out[1][0] = this->matrix_[2][0] * m[0][0] - this->matrix_[0][0] * m[2][0];
	out[2][0] = this->matrix_[0][0] * m[1][0] - this->matrix_[1][0] * m[0][0];
	return out;
};

double MatrixN::norm() const {
	if (this->getSize()[1] != kSingleColumn) {
		double out = 0;
		return out;
	}
	double out = 0;
	for (int i = 0; i < this->getSize()[0]; i++)
		out += this->matrix_[i][0] * this->matrix_[i][0];
	return out;
};

MatrixN MatrixN::T() const {
	MatrixN out(this->getSize()[1], this->getSize()[0]);
	for (int i = 0; i < this->getSize()[1]; i++)
		for (int j = 0; j < this->getSize()[0]; j++)
			out[i][j] = this->matrix_[j][i];
	return out;
};

void MatrixN::fill(const int fromRow, const int fromCol, MatrixN& m) {
	for (int i = fromRow; i < fromRow + m.getSize()[0]; i++)
		for (int j = fromCol; j < fromCol + m.getSize()[1]; j++)
			this->matrix_[i][j] = m[i - fromRow][j - fromCol];
};

MatrixN MatrixN::slice(const int fromRow, const int toRow, const int fromCol, const int toCol) {
	MatrixN out(toRow - fromRow, toCol - fromCol);
	for (int i = fromRow; i < toRow; i++)
		for (int j = fromCol; j < toCol; j++)
			out[i - fromRow][j - fromCol] = this->matrix_[i][j];
	return out;
};