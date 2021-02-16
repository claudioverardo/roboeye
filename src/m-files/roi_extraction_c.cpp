#include <queue>
#include <stack>

#include "mex.hpp"
#include "mexAdapter.hpp"

using namespace matlab::data;
using matlab::mex::ArgumentList;
using matlab::data::Array;
using matlab::data::CellArray;

typedef std::pair<int, int> ii;

class MexFunction : public matlab::mex::Function {
    // Internal state
    bool* image;     // Binary input image pointer
    
    int sizeI;       // Width pixel dimension
    int sizeJ;       // Height pixel dimension

    int componentsSize;
    std::vector<ii> *components;
    std::vector<ii> *tails;
    
    bool** visited;  // Visited input logical matrix pointer
    
    // Crosses and then diagonals
    /*const int add_i[8] = {  0, +1,  0, -1, -1, +1, +1, -1 };
    const int add_j[8] = { +1,  0, -1,  0, +1, +1, -1, -1 };*/
    
    // Diagonal and then diagonals
    const int add_i[8] = { -1, +1, +1, -1, -1,  0, +1,  0 };
    const int add_j[8] = { -1, -1, +1, +1,  0, -1,  0, +1 };
    
    public:
        void operator()(ArgumentList outputs, ArgumentList inputs) {
            // Validate input arguments
            // checkArguments(outputs, inputs);
            
            // Map inputs arguments in internal state variables
            // TODO: FIX WITH INTERNAL STATE POINTER CHECK DOCUMENTATION
            TypedArray<bool> inputImage = std::move(inputs[0]);
            this->image = getPointer(inputImage);
            mapInputArguments(&inputs);
            
            // Execute the roi extraction pipeline
            executePipeline();
            
            // Map outputs arguments in to matlab compatible types
            mapOutputArguments(&outputs);
            
            // Clear memory to next execution
            clearMemory();
        }
        
        void mapInputArguments(ArgumentList* p_inputs) {
            // Assign the input image
            /*this->inputImage = std::move((*p_inputs)[0]);
            this->image = getPointer(this->inputImage);*/
            
            // Assign the input image size
            this->sizeI = std::move((*p_inputs)[1])[0];
            this->sizeJ = std::move((*p_inputs)[2])[0];
            
            // Create local visited matrix
            this->visited = new bool*[this->sizeI];
            for (int i = 0; i < this->sizeI; i++) {
                this->visited[i] = new bool[this->sizeJ];
            }
            // Set the visited matrix to 0 (not visited)
            memset(*this->visited, 0, sizeof(*this->visited));
            
            // Create components dinamic array
            this->components = new std::vector<ii>[this->sizeI * this->sizeJ / 2];
            this->componentsSize = 0;
            
            // Create tails dinamic array
            this->tails = new std::vector<ii>[this->sizeI * this->sizeJ / 2];
        }
        
        void mapOutputArguments(ArgumentList* outputs) {
            // Create array factory to instatiate the matlab compatible arrays
            ArrayFactory factory;
            
            // Map components output matlab array
            CellArray componentsMatlab = factory.createCellArray({ this->componentsSize, 1 });
            for (int i = 0; i < this->componentsSize; i++) {
                TypedArray<double> singleComponent = factory.createArray<double>({ this->components[i].size(), 2 });
                for (int j = 0; j < this->components[i].size(); j++) {
                    singleComponent[j][0] = this->components[i][j].first;
                    singleComponent[j][1] = this->components[i][j].second;
                }
                componentsMatlab[i] = singleComponent;
            }
            (*outputs)[0] = std::move(componentsMatlab);
            
            // Populate tails for matlab output array
            CellArray tailsMatlab = factory.createCellArray({ this->componentsSize, 1 });
            for (int i = 0; i < this->componentsSize; i++) {
                TypedArray<double> singleTail = factory.createArray<double>({ this->tails[i].size(), 2 });
                for (int j = 0; j < this->tails[i].size(); j++) {
                    singleTail[j][0] = this->tails[i][j].first;
                    singleTail[j][1] = this->tails[i][j].second;
                }
                tailsMatlab[i] = singleTail;
            }
            (*outputs)[1] = std::move(tailsMatlab);
        }
        
        void clearMemory() {
            // Release components memory to prevent memory leaks
            for (int i = 0; i < this->componentsSize; i++) {
                this->components[i].clear();
            }
            delete [] this->components;
            
            // Release tails memory to prevent memory leaks
            for (int i = 0; i < this->componentsSize; i++) {
                this->tails[i].clear();
            }
            delete [] this->tails;
            
            // Release visited memory to prevent memory leaks
            for (int i = 0; i < this->sizeI; ++i) {
                delete [] this->visited[i];
            }
            delete [] this->visited;
        }
        
        void executePipeline() {
            // Foreach pixel
            for (int i = 0; i < this->sizeI; i++) {
                for (int j = 0; j < this->sizeJ; j++) {
                    // If is white and not visited we should explore the new connected component
                    if (*(this->image + getImageIndex(i, j)) == true &&
                        !visited[i][j]) {
                      
                        // Explore that new connected component and save points and tails
                        // bfs(i, j);
                        dfs(i, j);
                        
                        // Add to components tails the startpoint if there is a tail
                        if (checkTail(i, j)) {
                           this->tails[this->componentsSize].push_back({ j, i });
                        }
                        
                        // Check if component is not closed and delete it (minimum 100 pixels)
                        bool valid_component = true;
                        if (!checkConnectedComponent()) {
                            // Delete the lastest invalid component and tails
                            this->components[this->componentsSize].clear();
                            this->tails[this->componentsSize].clear();
                            valid_component = false;
                        }

                        // If is a valid component I can increse the number of connected components founded
                        if (valid_component) {
                            this->componentsSize++;
                        }
                    }
                }
            }
        }
        
        void dfs_rec(int u_i, int u_j) {
            visited[u_i][u_j] = true;
            this->components[this->componentsSize].push_back({ u_j, u_i });
            
            bool possibleTail = true;
            
            for (int i = 0; i < 8; i++) {
                int v_i = u_i + this->add_i[i];
                int v_j = u_j + this->add_j[i];

                if (checkBoundaries(v_i, v_j) &&                            // Inside the image bounds
                    !visited[v_i][v_j] &&                                   // Not visited anymore
                    *(this->image + getImageIndex(v_i, v_j)) == true) {     // There is a white pixel

                    visited[v_i][v_j] = true;
                    possibleTail = false;
                    dfs_rec(v_i, v_j);
                }
            }
            
            if (possibleTail) {
                // Check boundaries conditions
                if (checkTail(u_i, u_j)) {
                    this->tails[this->componentsSize].push_back({ u_j, u_i });
                }
            }
        }
        
        void bfs(int start_i, int start_j) {
            // Create queue
            std::queue<ii> q;
            
            // Push start node
            q.push({ start_i, start_j});
            visited[start_i][start_j] = true;
            
            // Until queue is empty
            while (!q.empty()) {
                // Extract the front coords
                ii u = q.front();
                int u_i = u.first;
                int u_j = u.second;
                
                // Remove the first element
                q.pop();
                
                // Add that point to the connected components set
                this->components[this->componentsSize].push_back({ u_j, u_i });
                
                bool possibleTail = true;
                
                for (int i = 0; i < 8; i++) {
                    int v_i = u_i + this->add_i[i];
                    int v_j = u_j + this->add_j[i];
                    
                    if (checkBoundaries(v_i, v_j) &&                            // Inside the image bounds
                        !visited[v_i][v_j] &&                                   // Not visited anymore
                        *(this->image + getImageIndex(v_i, v_j)) == true) {     // There is a white pixel
                        
                        visited[v_i][v_j] = true;
                        possibleTail = false;
                        q.push({ v_i, v_j });
                    }
                }
                
                if (possibleTail) {
                    // Check boundaries conditions
                    if (checkTail(u_i, u_j)) {
                        this->tails[this->componentsSize].push_back({ u_j, u_i });
                    }
                }
            }
        }
        
         void dfs(int start_i, int start_j) {
            // Create queue
            std::stack<ii> q;
            
            // Push start node
            q.push({ start_i, start_j});
            visited[start_i][start_j] = true;
            
            // Until queue is empty
            while (!q.empty()) {
                // Extract the front coords
                ii u = q.top();
                int u_i = u.first;
                int u_j = u.second;
                
                // Remove the first element
                q.pop();
                
                // Add that point to the connected components set
                this->components[this->componentsSize].push_back({ u_j, u_i });
                
                bool possibleTail = true;
                
                for (int i = 0; i < 8; i++) {
                    int v_i = u_i + this->add_i[i];
                    int v_j = u_j + this->add_j[i];
                    
                    if (checkBoundaries(v_i, v_j) &&                            // Inside the image bounds
                        !visited[v_i][v_j] &&                                   // Not visited anymore
                        *(this->image + getImageIndex(v_i, v_j)) == true) {     // There is a white pixel
                        
                        visited[v_i][v_j] = true;
                        possibleTail = false;
                        q.push({ v_i, v_j });
                    }
                }
                
                if (possibleTail) {
                    // Check boundaries conditions
                    if (checkTail(u_i, u_j)) {
                        this->tails[this->componentsSize].push_back({ u_j, u_i });
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
        
        bool checkConnectedComponent() {
             if (this->tails[this->componentsSize].size() > 2 ||        // Tails Threshold
                 this->tails[this->componentsSize].size() == 1 ||       // Tails Threeshold
                 this->components[this->componentsSize].size() < 35   // Minumum Pixel Threeshold
                 /* TODO: Manhattan distance ( 
                    size(component{1, 2}, 1) == 2 &&
                    sum(abs(component{1, 2}(1, :)' - component{1, 2}(2, :)')) > 8  // Manhattan distance Threeshold
                 )*/) {
                return false;
             }
             return true;
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