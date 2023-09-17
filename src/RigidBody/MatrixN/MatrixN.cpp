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
	MatrixN out(m.getSize()[0], m.getSize()[1]);
	out.matrix_ = m.matrix_;
	return out;
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

MatrixN MatrixN::operator/(const MatrixN& v) const {
	MatrixN out(this->getSize()[0], kSingleColumn);
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