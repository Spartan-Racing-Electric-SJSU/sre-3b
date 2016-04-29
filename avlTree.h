//http://www.zentut.com/c-tutorial/c-avl-tree/


#ifndef AVLTREE_H_INCLUDED
#define AVLTREE_H_INCLUDED

typedef struct AVLNode
{
	//Message Metadata -----------------------------------------------------
	//int data;
	ubyte4 id;           /**< ID for CAN communication             */
	ubyte1 data[8];      /**< data buffer                          */
	//ubyte1 length;       /**< number of words in transmit buffer   */
	//ubyte1 id_format;    /**< standard or extended format          */
	ubyte4 timeBetweenMessages_Min;
	ubyte4 timeBetweenMessages_Max;
	ubyte1 lastMessage_data[8];
	ubyte4 lastMessage_timeStamp;

	//Tree stuff -----------------------------------------------------
	struct AVLNode*  left;
	struct AVLNode*  right;
	int      height;
} AVLNode;

AVLNode* find(int e, AVLNode *t);
AVLNode* find_min(AVLNode *t);
AVLNode* find_max(AVLNode *t);
AVLNode* insert(int data, AVLNode *t);
void display_avl(AVLNode* t);
int get(AVLNode* n);
void dispose(AVLNode* t);
AVLNode* delete(int data, AVLNode *t);


/*
//C AVL tree program main.c
#include <stdio.h>
#include "avltree.h"

int main()
{
AVLNode *t , *p;
int i;
int j = 0;
const int max = 10;

printf("--- C AVL Tree Demo  ---\n");

t = NULL;

printf("Insert: ");
for( i = 0; i < max; i++, j = ( j + 7 ) % max )
{

t = insert( j, t );
printf("%d ",j);

}
printf(" into the tree\n\n");

display_avl(t);

dispose(t);

return 0;
}
*/



#endif // AVLTREE_H_INCLUDED