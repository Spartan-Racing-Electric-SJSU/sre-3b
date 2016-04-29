//Original source: http://www.geeksforgeeks.org/avl-tree-set-1-insertion/
#include <stdlib.h>

#include "IO_Driver.h"
#include "IO_CAN.h"

#include "mathFunctions.h"
#include "avlTree.h"


// A utility function to get height of the tree
int height(avlNode *N)
{
    if (N == NULL)
        return 0;
    return N->height;
}
 
/* Helper function that allocates a new node with the given key and
    NULL left and right pointers. */
avlNode* newNode(IO_CAN_DATA_FRAME* newMessage, ubyte4* timeBetweenMessages_Min, ubyte4* timeBetweenMessages_Max, ubyte4* lastMessage_timeStamp)
{
    avlNode* me = (avlNode*)malloc(sizeof(struct _avlNode));
    //node->key    = key;
	me->left   = NULL;
	me->right  = NULL;
	me->height = 1;  // new node is initially added at leaf

	//Copy the CAN message into this node's memory
	//Reference: http://stackoverflow.com/questions/9127246/copy-struct-to-struct-in-c
	me->lastMessage = *newMessage; //dereference the message - we want to copy the data of the message, not it's memory address

	//Update the message data
	me->timeBetweenMessages_Min = *timeBetweenMessages_Min; 
	me->timeBetweenMessages_Max = *timeBetweenMessages_Max;
	me->lastMessage_timeStamp = *lastMessage_timeStamp;


    return(node);
}

avlNode* insert(avlNode* node, IO_CAN_DATA_FRAME* newMessage, ubyte4* timeBetweenMessages_Min, ubyte4* timeBetweenMessages_Max, ubyte4* lastMessage_timeStamp)
{
    /* 1.  Perform the normal BST rotation */
    if (node == NULL)
        return(newNode(newMessage, timeBetweenMessages_Min, timeBetweenMessages_Max, lastMessage_timeStamp));
 
    if (newMessage->id < node->canMessage->id)
        node->left  = insert(node->left, newMessage, timeBetweenMessages_Min, timeBetweenMessages_Max, lastMessage_timeStamp);
	else
		node->right = insert(node->right, newMessage, timeBetweenMessages_Min, timeBetweenMessages_Max, lastMessage_timeStamp);

	/* 2. Update height of this ancestor node */
	node->height = max(height(node->left), height(node->right)) + 1;

	/* 3. Get the balance factor of this ancestor node to check whether
	this node became unbalanced */
	int balance = getBalance(node);

	// If this node becomes unbalanced, then there are 4 cases

	// Left Left Case
	if (balance > 1 && newMessage->id < node->left->lastMessage->id)
		return rightRotate(node);

	// Right Right Case
	if (balance < -1 && newMessage->id > node->right->lastMessage->id)
		return leftRotate(node);

	// Left Right Case
	if (balance > 1 && newMessage->id > node->left->lastMessage->id)
	{
		node->left = leftRotate(node->left);
		return rightRotate(node);
	}

    // Right Left Case
    if (balance < -1 && newMessage->id < node->right->lastMessage->id)
    {
        node->right = rightRotate(node->right);
        return leftRotate(node);
    }
 
    /* return the (unchanged) node pointer */
    return node;
}

/*
struct node* deleteNode(struct node* root, int key)
{
	// STEP 1: PERFORM STANDARD BST DELETE

	if (root == NULL)
		return root;

	// If the key to be deleted is smaller than the root's key,
	// then it lies in left subtree
	if (key < root->key)
		root->left = deleteNode(root->left, key);

	// If the key to be deleted is greater than the root's key,
	// then it lies in right subtree
	else if (key > root->key)
		root->right = deleteNode(root->right, key);

	// if key is same as root's key, then This is the node
	// to be deleted
	else
	{
		// node with only one child or no child
		if ((root->left == NULL) || (root->right == NULL))
		{
			struct node *temp = root->left ? root->left : root->right;

			// No child case
			if (temp == NULL)
			{
				temp = root;
				root = NULL;
			}
			else // One child case
				*root = *temp; // Copy the contents of the non-empty child

			free(temp);
		}
		else
		{
			// node with two children: Get the inorder successor (smallest
			// in the right subtree)
			struct node* temp = minValueNode(root->right);

			// Copy the inorder successor's data to this node
			root->key = temp->key;

			// Delete the inorder successor
			root->right = deleteNode(root->right, temp->key);
		}
	}

	// If the tree had only one node then return
	if (root == NULL)
		return root;

	// STEP 2: UPDATE HEIGHT OF THE CURRENT NODE
	root->height = max(height(root->left), height(root->right)) + 1;

	// STEP 3: GET THE BALANCE FACTOR OF THIS NODE (to check whether
	//  this node became unbalanced)
	int balance = getBalance(root);

	// If this node becomes unbalanced, then there are 4 cases

	// Left Left Case
	if (balance > 1 && getBalance(root->left) >= 0)
		return rightRotate(root);

	// Left Right Case
	if (balance > 1 && getBalance(root->left) < 0)
	{
		root->left = leftRotate(root->left);
		return rightRotate(root);
	}

	// Right Right Case
	if (balance < -1 && getBalance(root->right) <= 0)
		return leftRotate(root);

	// Right Left Case
	if (balance < -1 && getBalance(root->right) > 0)
	{
		root->right = rightRotate(root->right);
		return leftRotate(root);
	}

	return root;
}
*/

// A utility function to right rotate subtree rooted with y
// See the diagram given above.
avlNode *rightRotate(avlNode *y)
{
	avlNode *x = y->left;
	avlNode *T2 = x->right;

	// Perform rotation
	x->right = y;
	y->left = T2;

	// Update heights
	y->height = max(height(y->left), height(y->right)) + 1;
	x->height = max(height(x->left), height(x->right)) + 1;

	// Return new root
	return x;
}

// A utility function to left rotate subtree rooted with x
// See the diagram given above.
avlNode *leftRotate(avlNode *x)
{
	avlNode *y = x->right;
	avlNode *T2 = y->left;

	// Perform rotation
	y->left = x;
	x->right = T2;

	//  Update heights
	x->height = max(height(x->left), height(x->right)) + 1;
	y->height = max(height(y->left), height(y->right)) + 1;

	// Return new root
	return y;
}

// Get Balance factor of node N
int getBalance(avlNode *N)
{
	if (N == NULL)
		return 0;
	return height(N->left) - height(N->right);
}

// A utility function to print preorder traversal of the tree.
// The function also prints height of every node
/*
void preOrder(avlNode *root)
{
    if(root != NULL)
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