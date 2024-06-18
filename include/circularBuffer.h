template <typename T, size_t bufferSize>
class CircularBuffer {
private:
    // The underlying array to store the buffer elements
    T buffer[bufferSize];

    // The current index in the buffer
    uint32_t index;

public:
    // Constructor to initialize the index to 0
    CircularBuffer() : index(0) {}

    // Add a new element to the buffer
    void add(T item) {
        // Store the new element at the current index
        buffer[index] = item;
        // Increment the index, wrapping around to 0 if necessary
        index = (index + 1) % bufferSize;
    }

    // Overload the [] operator to access elements in the buffer
    T& operator[](size_t i) {
        // Calculate the index of the element to return
        // This formula ensures wrapping around the buffer
        return buffer[(index - 1 - i + bufferSize) % bufferSize];
    }
};

