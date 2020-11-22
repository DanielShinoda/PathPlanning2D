#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <vector>
#include <unordered_map>
#include <math.h>
#include <limits>
#include <list>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <chrono>

template<class KeyType, class ValueType, class Hash = std::hash<KeyType>> class HashMap {
public:
    // Для удобства и во избежании загромождения кода
    using iterator = typename std::list<std::pair<const KeyType, ValueType>>::iterator;
    using const_iterator = typename std::list<std::pair<const KeyType,
        ValueType>>::const_iterator;
    using iterator_vector = typename std::vector<std::list<iterator>>;

private:
    std::list<std::pair<const KeyType, ValueType>> pairs;

    std::vector<std::list<typename std::list<std::pair<const KeyType, ValueType>>::iterator>>
        hashTable;

    Hash hash_; // Наша хеш-функция.

    size_t capacity = 0;

    size_t amount = 0;

    size_t myHash(KeyType elem) const {
        return hash_(elem) % capacity;
    }

    void createTable(const int capacity_ = 16) {
        capacity = capacity_;
        hashTable = iterator_vector(capacity);
    }

    // Если ключ раньше не встречался в таблице
    // Вызываем этот метод
    void push_back(std::pair<KeyType, ValueType> elem, int hashed) {
        pairs.push_back(elem);
        auto it = pairs.end();
        --it;
        hashTable[hashed].push_back(it);
    }

    // Рехешинг таблицы
    void rehash() {
        if (amount < capacity) return;

        std::list<std::pair<KeyType, ValueType>> temporary_list;
        for (auto& p : pairs) temporary_list.push_back(std::make_pair(p.first, p.second));

        pairs.clear();
        hashTable.clear();
        createTable(capacity * 2);
        amount = 0;
        for (auto& el : temporary_list) insert(el);
        temporary_list.clear();
    }

public:
    // Конструктор по умолчанию
    HashMap(Hash hasherObj = Hash()) : hash_(hasherObj) { createTable(); }

    // Конструктор, принимающий итераторы на начало и конец последовательности
    template<typename iter>
    HashMap(iter begin, iter end, Hash hasher = Hash()) : hash_(hasher) {
        createTable();
        while (begin++ != end) insert(*begin);
    }

    // Конструкор, принимающий std::initializer_list
    HashMap(std::initializer_list<std::pair<KeyType, ValueType>> list,
        Hash hasher = Hash())
        : hash_(hasher) {
        createTable();
        for (auto& p : list) insert(p);
    }

    void insert(std::pair<KeyType, ValueType> elem) {
        size_t hashed = myHash(elem.first);
        // Проверка на наличие элемента в текущей ячейке хеша:
        for (auto& p : hashTable[hashed])
            if (p->first == elem.first)
                return;
        // Так как элемент не встретился:
        push_back(elem, hashed);
        ++amount; // добавили элемент.
        rehash(); // if elemnts amount * 2 < capacity: rehash
    }

    void erase(KeyType key) {
        auto iter = hashTable[myHash(key)].begin();
        for (; iter != hashTable[myHash(key)].end(); ++iter) {
            if ((*iter)->first == key) {
                hashTable[myHash(key)].erase(iter);
                pairs.erase(*iter);
                --amount;
                return;
            }
        }
    }

    void clear() {
        for (auto& elem : pairs) hashTable[myHash(elem.first)].clear();
        pairs.clear();
        amount = 0;
    }

    HashMap& operator =(const HashMap& other) {
        clear();
        hash_ = other.hash_function;
        createTable(other.capacity);
        for (const auto& elem : other) insert(elem);
        return *this;
    }

    iterator find(KeyType key) {
        size_t hashed = myHash(key);
        for (auto& element : hashTable[hashed])
            if (element->first == key) return element;
        return end();
    }

    const_iterator find(KeyType key) const {
        size_t hashed = myHash(key);
        for (auto& element : hashTable[hashed])
            if (element->first == key) return element;
        return end();
    }

    ValueType& operator[] (KeyType key) {
        insert(std::make_pair(key, ValueType()));
        auto iter = find(key);
        return iter->second;
    }

    const ValueType& at(KeyType key) const {
        auto iter = find(key);
        if (iter == end()) throw std::out_of_range("Out of range!");
        return iter->second;
    }

    size_t size() const { return amount; }

    bool empty() const { return amount == 0; }

    Hash hash_function() const { return hash_; }

    iterator begin() {
        return iterator(pairs.begin());
    }

    iterator end() {
        return iterator(pairs.end());
    }

    const_iterator begin() const {
        return pairs.begin();
    }

    const_iterator end() const {
        return pairs.end();
    }

    ~HashMap() {
        clear();
    }
};

class Search
{
    public:
        Search();
        virtual ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    protected:
        //Using own Hash table class for OPEN and CLOSED
        HashMap<int, Node> OPEN, CLOSED;

        //Get Heuristic function for several cases
        virtual double getHeuristic(int a1, int b1, const Map& map, const EnvironmentOptions& options);
        virtual double getHeuristic(int x1, int y1, int x2, int y2, const EnvironmentOptions& options);
        virtual Node argmin(const EnvironmentOptions& options);
        virtual std::unordered_map<int, Node> getSuccessors(Node s, const Map& map, const EnvironmentOptions& options);
        virtual Node changeParent(Node currentNode, Node parentNode, const Map& map, const EnvironmentOptions& options);
        virtual void makePrimaryPath(Node currentNode);
        virtual void makeSecondaryPath();
        SearchResult sresult; //This will store the search result
        std::list<Node> lppath, hppath; //
        
        //CODE HERE to define other members of the class
};
#endif
