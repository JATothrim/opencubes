#include "../include/newCache.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>

CacheReader::CacheReader(const std::string& path)
    : path_(path), fileDescriptor_(0), fileSize_(0), fileLoaded_(false), dummyHeader{0, 0, 0, 0}, header(&dummyHeader), shapes(0) {
    if (loadFile(path) != 0) {
		std::printf("failed to load data from \"%s\"", path.c_str());
    }
}
void CacheReader::printHeader() {
    if (fileLoaded_) {
        std::printf("magic: %x ", header->magic);
        std::printf("n: %d ", header->n);
        std::printf("numShapes: %d ", header->numShapes);
        std::printf("numPolycubes: %ld\n", header->numPolycubes);
    } else {
        std::printf("no file loaded!\n");
    }
}

int CacheReader::printShapes(void) {
    if (fileLoaded_) {
        for (uint64_t i = 0; i < header->numShapes; i++) {
            std::printf("%d\t%d\t%d\n", shapes[i].dim0, shapes[i].dim1, shapes[i].dim2);
        }
        return 1;
    }
    return 0;
}

int CacheReader::loadFile(std::string path) {
    path_ = path;
    fileDescriptor_ = open(path.c_str(), O_RDONLY);

    if (fileDescriptor_ == -1) {
		std::printf("error opening file\n");
        return 1;
    }

    // get filesize
    fileSize_ = lseek(fileDescriptor_, 0, SEEK_END);
    lseek(fileDescriptor_, 0, SEEK_SET);

    // memory map file
    filePointer = (uint8_t*)mmap(NULL, fileSize_, PROT_READ, MAP_PRIVATE, fileDescriptor_, 0);
    if (filePointer == MAP_FAILED) {
        // error handling
        std::printf("errorm mapping file memory");
        close(fileDescriptor_);
        return 2;
    }

    header = (Header*)(filePointer);
    shapes = (ShapeEntry*)(filePointer + sizeof(Header));
    data = (char*)(filePointer + sizeof(Header) + header->numShapes * sizeof(ShapeEntry));

    fileLoaded_ = true;

    return 0;
}

CacheReader::~CacheReader() {
    // unmap file from memory
    if (munmap(filePointer, fileSize_) == -1) {
        // error handling
        std::printf("error unmapping file\n");
    }

    // close file descriptor
    close(fileDescriptor_);
    fileLoaded_ = false;
}
/*
int main(int argc, char** argv) {
    CacheReader cr(argv[1]);
    printf("----------\n");
    cr.printShapes();
    printf("---------\n");
    printf("%d\n", cr.header->numShapes);
    return 0;
}
*/
