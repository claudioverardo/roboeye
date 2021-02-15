#include <queue>

#include "mex.hpp"
#include "mexAdapter.hpp"

using namespace matlab::data;
using matlab::mex::ArgumentList;
using matlab::data::Array;

typedef std::pair<int, int> ii;

class MexFunction : public matlab::mex::Function {
    bool* image;    // Binary input image pointer
    bool** visited; // Visited dinamic matrix
    std::vector<ii> components; 
    std::vector<ii> tails;
    int sizeI;      // Width pixel dimension
    int sizeJ;      // Height pixel dimension
    
    public:
        void operator()(ArgumentList outputs, ArgumentList inputs) {
            // Validate input arguments
            // checkArguments(outputs, inputs);
            
            // Assign the input image
            TypedArray<bool> matrix = std::move(inputs[0]);
            this->image = getPointer(matrix);
            
            // Assign the input image size
            this->sizeI = std::move(inputs[1])[0];
            this->sizeJ = std::move(inputs[2])[0];
            
            // Save startI and startJ coords
            int startI = std::move(inputs[3])[0];
            int startJ = std::move(inputs[4])[0];
            
            // Initialize the visited matrix
            this->visited = new bool*[this->sizeI];
            for(int i = 0; i < this->sizeI; i++) {
                this->visited[i] = new bool[this->sizeJ];
            }
            
            bfs(startI, startJ);
            
            ArrayFactory factory;
            
            // Populate components for matlab output array
            TypedArray<int> componentsMatlab = factory.createArray<int>({ this->components.size(), 2 });
            for (int i = 0; i < this->components.size(); i++) {
                componentsMatlab[i][0] = this->components[i].first;
                componentsMatlab[i][1] = this->components[i].second;
            }
            outputs[0] = componentsMatlab;
            
            // Populate tails for matlab output array
            TypedArray<int> tailsMatlab = factory.createArray<int>({ this->tails.size(), 2 });
            for (int i = 0; i < this->tails.size(); i++) {
                tailsMatlab[i][0] = this->tails[i].first;
                tailsMatlab[i][1] = this->tails[i].second;
            }
            outputs[1] = tailsMatlab;
        }
        
        void bfs(int start_i, int start_j) {
            // Crosses and then diagonals
            int add_i[8] = {  0, +1,  0, -1, -1, +1, +1, -1 };
            int add_j[8] = { +1,  0, -1,  0, +1, +1, -1, -1 };
    
            // Create queue
            std::queue<ii> q;
            
            // Push start node
            q.push({ start_i, start_j });
            
            // Until queue is empty
            while (!q.empty()) {
                // Extract the front coords
                ii u = q.front();
                int u_i = u.first;
                int u_j = u.second;
                
                // Remove the first element
                q.pop();
                
                // Signed that pixel as visited
                visited[u_i][u_j] = true;
                this->components.push_back({ u_j, u_i });
                
                bool possibleTail = true;
                
                for (int i = 0; i < 8; i++) {
                    int v_i = u_i + add_i[i];
                    int v_j = u_j + add_j[i];
                    
                    if (checkBoundaries(v_i, v_j) &&
                        !visited[v_i][v_j] &&
                        *(this->image + getImageIndex(v_i, v_j)) == true) {
                        possibleTail = false;
                        q.push({ v_i, v_j });
                    }
                }
                
                if (possibleTail) {
                    // Check boundaries conditions
                    if (checkTail(u_i, u_j)) {
                        this->tails.push_back({ u_j, u_i });
                    }
                }
            }
        }
        
        bool checkBoundaries(int i, int j) {    
            return ((i >= 0 && i < this->sizeI) && 
                    (j >= 0 && j < this->sizeJ));
        }
        
        int getImageIndex(int i, int j) {
            return i + j * this->sizeI;
        }

        bool checkTail(int pixel_i, int pixel_j) {
            int filled = 0;
    
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (checkBoundaries(pixel_i + i, pixel_j + j) == 1 && 
                        *(this->image + getImageIndex(pixel_i + i, pixel_j + j)) == true) {
                        filled = filled + 1;
                    }
                }
            }

            if (filled < 3) {
                return true;
            } else if (filled == 3) {
                bool isOk = false;

                // Corner case top left
                int partial_filled = 0;
                for (int i = -1; i <= 0; i++) {
                    for (int j = -1; j <= 0; j++) {
                        if (checkBoundaries(pixel_i + i, pixel_j + j) == 1 &&
                            *(this->image + getImageIndex(pixel_i + i, pixel_j + j)) == true) {
                            partial_filled = partial_filled + 1;
                        }
                    }
                }
                if (partial_filled == 3) {
                    isOk = true;
                }

                // Corner case top right
                partial_filled = 0;
                for (int i = -1; i <= 0; i++) {
                    for (int j = 0; j <= 1; j++) {
                        if (checkBoundaries(pixel_i + i, pixel_j + j) == 1 &&
                            *(this->image + getImageIndex(pixel_i + i, pixel_j + j)) == true) {
                            partial_filled = partial_filled + 1;
                        }
                    }
                }
                if (partial_filled == 3) {
                    isOk = true;
                }
                
                // Corner case bottom left
                partial_filled = 0;
                for (int i = 0; i <= 1; i++) {
                    for (int j = -1; j <= 0; j++) {
                        if (checkBoundaries(pixel_i + i, pixel_j + j) == 1 &&
                            *(this->image + getImageIndex(pixel_i + i, pixel_j + j)) == true) {
                            partial_filled = partial_filled + 1;
                        }
                    }
                }
                if (partial_filled == 3) {
                    isOk = true;
                }

                // Corner case bottom right
                partial_filled = 0;
                for (int i = 0; i <= 1; i++) {
                    for (int j = 0; j <= 1; j++) {
                        if (checkBoundaries(pixel_i + i, pixel_j + j) == 1 &&
                            *(this->image + getImageIndex(pixel_i + i, pixel_j + j)) == true) {
                            partial_filled = partial_filled + 1;
                        }
                    }
                }
                if (partial_filled == 3) {
                    isOk = true;
                }

                // Corner case perfect L 
                if (checkBoundaries(pixel_i - 1, pixel_j) &&
                    checkBoundaries(pixel_i, pixel_j - 1) && 
                    *(this->image + getImageIndex(pixel_i - 1, pixel_j)) == true &&
                    *(this->image + getImageIndex(pixel_i, pixel_j - 1)) == true) {
                    isOk = false;
                }

                // Corner case perfect L 
                if (checkBoundaries(pixel_i - 1, pixel_j) &&
                    checkBoundaries(pixel_i, pixel_j + 1) &&
                    *(this->image + getImageIndex(pixel_i - 1, pixel_j)) == true &&
                    *(this->image + getImageIndex(pixel_i, pixel_j + 1)) == true) {
                    isOk = false;
                }

                // Corner case perfect L 
                if (checkBoundaries(pixel_i + 1, pixel_j) &&
                    checkBoundaries(pixel_i, pixel_j - 1) &&
                    *(this->image + getImageIndex(pixel_i + 1, pixel_j)) == true &&
                    *(this->image + getImageIndex(pixel_i, pixel_j - 1)) == true) {
                    isOk = false;
                }

                // Corner case perfect L 
                if (checkBoundaries(pixel_i + 1, pixel_j) &&
                    checkBoundaries(pixel_i, pixel_j + 1) &&
                    *(this->image + getImageIndex(pixel_i + 1, pixel_j)) == true &&
                    *(this->image + getImageIndex(pixel_i, pixel_j + 1)) == true) {
                    isOk = false;
                }

                if (isOk) {
                    return true;
                }
            }

            return false;
        }
        
        void checkArguments(ArgumentList outputs, ArgumentList inputs) {
            // Get pointer to engine
            std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

            // Get array factory
            ArrayFactory factory;
            
            /*if (inputs.size() != 2) {
                matlabPtr->feval(
                    u"error", 
                    0, 
                    std::vector<Array>({ factory.createScalar("Require 2 inputs")})
                );
            }*/

            // Check first input argument
            /*if (inputs[0].getType() != ArrayType::DOUBLE ||
                inputs[0].getType() == ArrayType::COMPLEX_DOUBLE ||
                inputs[0].getNumberOfElements() != 1)
            {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("First input must be scalar double") }));
            }

            // Check second input argument
            if (inputs[1].getType() != ArrayType::DOUBLE ||
                inputs[1].getType() == ArrayType::COMPLEX_DOUBLE)
            {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("Input must be double array") }));
            }
            
            // Check number of outputs
            if (outputs.size() > 1) {
                matlabPtr->feval(u"error",
                    0,
                    std::vector<Array>({ factory.createScalar("Only one output is returned") }));
            }*/
        }
        
        template <typename T>
        inline T* toPointer(const matlab::data::TypedIterator<T>& it) MW_NOEXCEPT {
            static_assert(std::is_arithmetic<T>::value && !std::is_const<T>::value,
                "Template argument T must be a std::is_arithmetic and non-const type.");
            return it.operator->();
        }
        
        template <typename T>
        inline T* getPointer(matlab::data::TypedArray<T>& arr) MW_NOEXCEPT {
            static_assert(std::is_arithmetic<T>::value, "Template argument T must be a std::is_arithmetic type.");
            return toPointer(arr.begin());
        }
        
        template <typename T>
        inline const T* getPointer(const matlab::data::TypedArray<T>& arr) MW_NOEXCEPT {
            return getPointer(const_cast<matlab::data::TypedArray<T>&>(arr));
        }
};