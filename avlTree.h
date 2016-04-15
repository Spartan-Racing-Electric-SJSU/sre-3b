#include "IO_Driver.h"
//Source: http://www.geeksforgeeks.org/avl-tree-set-1-insertion/

// An AVL tree node
typedef struct _avlNode
{
	//Structure variables
	//int key;
	avlNode *left;
	avlNode *right;
	int height;

	//Data values
	IO_CAN_DATA_FRAME canMessage;
	ubyte4 timeBetweenMessages_Min;
	ubyte4 timeBetweenMessages_Max;
	ubyte1 lastMessage_data[8];
	ubyte4 lastMessage_timeStamp;
}
avlNode;

avlNode* insert(avlNode* node, int key);
avlNode* newNode(IO_CAN_DATA_FRAME canMessage, ubyte4 timeBetweenMessages_Min, ubyte4 timeBetweenMessages_Max, ubyte4 lastMessage_timeStamp);

// A utility function to print preorder traversal of the tree.
// The function also prints height of every node
/*
void preOrder(avlNode *root)
{
	if (root != NULL)
	{
		printf("%d ", root->key);
		preOrder(root->left);
		preOrder(root->right);
	}
}
*/

/* Drier program to test above function*/
//int main()
//{
//  avlNode *root = NULL;
// 
//  /* Constructing tree given in the above figure */
//  root = insert(root, 10);
//  root = insert(root, 20);
//  root = insert(root, 30);
//  root = insert(root, 40);
//  root = insert(root, 50);
//  root = insert(root, 25);
// 
//  /* The constructed AVL Tree would be
//            30
//           /  \
//         20   40
//        /  \     \
//       10  25    50
//  */
// 
//  printf("Pre order traversal of the constructed AVL tree is \n");
//  preOrder(root);
// 
//  return 0;
//}