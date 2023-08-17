#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <random>


void convRows(std::vector<std::vector<unsigned char>>& input,
            std::vector<std::vector<int>>& output, std::vector<int>& kernel){
    int nrows = input.size();
    int ncols = input[0].size();
    int len_ker = kernel.size();
    int half_ker = len_ker / 2;

    for (int r = 0; r < nrows; r++) {
        for (int c = 0; c < ncols; c++) {
            int sum_ = 0;
            for (int i = 0; i < len_ker; i++) {
                int input_col = c + i - half_ker;
                // assume zero padding
                if (input_col >= 0 && input_col < ncols) {
                    sum_ += (int)input[r][input_col] * kernel[i];
                }
            }
            output[r][c] = sum_; 
        }
    }
}

void Convolution(const std::vector<std::vector<int>>& img, std::vector<std::vector<int>>& out,
                const std::vector<std::vector>& kernel, const bool padding){
                
                int row = img.size();
                int col = img[0].size();
                int n_ker = kernel.size();

                for (int x = 0 ; x < row ; x++) {
                    for (int y = 0 ; y < col ; y++) {
                        float temp = 0.0;
                        for (int i = -n_ker/2 ; i <= n_ker/2 ; i++) {
                            int r = x+i;
                            if (r < 0){ 
                                if(padding)
                                    continue;
                                r = 0;
                            }
                            if (r >= row) {
                                if(padding)
                                    continue;
                                r = row-1;
                            }
                            for (int j = -n_ker/2 ; j<= n_ker/2 ; j++) {
                                int c = y+j;
                                if (c < 0) c = 0;
                                if (c >= col) c = col-1;
                                temp += img[r][c]*kernel[i+n_ker/2][j+n_ker/2];
                            }
                        }
                    }
			    output[x][y] = temp;
		    }
}

void createImage(int height, int width, std::vector<std::vector<unsigned char>>& matrix){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, 255); // Assuming the data type supports values from 0 to 255

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            matrix[i][j] = static_cast<unsigned char>(dist(gen));
        }
    }
}