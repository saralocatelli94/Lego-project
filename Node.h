/*
 * Node.h
 *
 *  Created on: 23 set 2017
 *      Author: sara
 */

#ifndef NODE_H_
#define NODE_H_

class Node {
public:
    Node();
    Node(int v);
    virtual ~Node();
    bool hasPrev();
    Node* getPrev();
    void setPrev(Node* newPrev);
    int getValue();
    void setValue(int val);
private:
    Node* prev;
    int value;
};

#endif /* NODE_H_ */
