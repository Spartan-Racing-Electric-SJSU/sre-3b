//http://www.zentut.com/c-tutorial/c-avl-tree/

#include <stdio.h>
#include "avlTree.h"

/*
find a specific node's key in the tree
*/
AVLNode* find(int messageID, AVLNode* t)
{
	if (t == NULL)
		return NULL;
	if (messageID < t->data)
		return find(messageID, t->left);
	else if (messageID > t->data)
		return find(messageID, t->right);
	else
		return t;
}

/*
find minimum node's key
*/
AVLNode* find_min(AVLNode* t)
{
	if (t == NULL)
		return NULL;
	else if (t->left == NULL)
		return t;
	else
		return find_min(t->left);
}

/*
find maximum node's key
*/
AVLNode* find_max(AVLNode* t)
{
	if (t != NULL)
		while (t->right != NULL)
			t = t->right;

	return t;
}

/*
get the height of a node
*/
static int height(AVLNode* n)
{
	if (n == NULL)
		return -1;
	else
		return n->height;
}

/*
get maximum value of two integers
*/
static int max(int l, int r)
{
	return l > r ? l : r;
}

/*
perform a rotation between a k2 node and its left child

note: call single_rotate_with_left only if k2 node has a left child
*/

static AVLNode* single_rotate_with_left(AVLNode* k2)
{
	AVLNode* k1 = NULL;

	k1 = k2->left;
	k2->left = k1->right;
	k1->right = k2;

	k2->height = max(height(k2->left), height(k2->right)) + 1;
	k1->height = max(height(k1->left), k2->height) + 1;
	return k1; /* new root */
}

/*
perform a rotation between a node (k1) and its right child

note: call single_rotate_with_right only if
the k1 node has a right child
*/

static AVLNode* single_rotate_with_right(AVLNode* k1)
{
	AVLNode* k2;

	k2 = k1->right;
	k1->right = k2->left;
	k2->left = k1;

	k1->height = max(height(k1->left), height(k1->right)) + 1;
	k2->height = max(height(k2->right), k1->height) + 1;

	return k2;  /* New root */
}

/*

perform the left-right double rotation,

note: call double_rotate_with_left only if k3 node has
a left child and k3's left child has a right child
*/

static AVLNode* double_rotate_with_left(AVLNode* k3)
{
	/* Rotate between k1 and k2 */
	k3->left = single_rotate_with_right(k3->left);

	/* Rotate between K3 and k2 */
	return single_rotate_with_left(k3);
}

/*
perform the right-left double rotation

notes: call double_rotate_with_right only if k1 has a
right child and k1's right child has a left child
*/



static AVLNode* double_rotate_with_right(AVLNode* k1)
{
	/* rotate between K3 and k2 */
	k1->right = single_rotate_with_left(k1->right);

	/* rotate between k1 and k2 */
	return single_rotate_with_right(k1);
}

/*
insert a new node into the tree
*/
AVLNode* insert(int e, AVLNode* t)
{
	if (t == NULL)
	{
		/* Create and return a one-node tree */
		t = (AVLNode*)malloc(sizeof(AVLNode));
		if (t == NULL)
		{
			fprintf(stderr, "Out of memory!!! (insert)\n");
			exit(1);
		}
		else
		{
			t->data = e;
			t->height = 0;
			t->left = t->right = NULL;
		}
	}
	else if (e < t->data)
	{
		t->left = insert(e, t->left);
		if (height(t->left) - height(t->right) == 2)
			if (e < t->left->data)
				t = single_rotate_with_left(t);
			else
				t = double_rotate_with_left(t);
	}
	else if (e > t->data)
	{
		t->right = insert(e, t->right);
		if (height(t->right) - height(t->left) == 2)
			if (e > t->right->data)
				t = single_rotate_with_right(t);
			else
				t = double_rotate_with_right(t);
	}
	/* Else X is in the tree already; we'll do nothing */

	t->height = max(height(t->left), height(t->right)) + 1;
	return t;
}


/*
data data of a node
*/
int get(AVLNode* n)
{
	return n->data;
}

/*
Recursively display AVL tree or subtree
*/
void display_avl(AVLNode* t)
{
	if (t == NULL)
		return;
	printf("%d", t->data);

	if (t->left != NULL)
		printf("(L:%d)", t->left->data);
	if (t->right != NULL)
		printf("(R:%d)", t->right->data);
	printf("\n");

	display_avl(t->left);
	display_avl(t->right);
}

/*
remove all nodes of an AVL tree
*/
void dispose(AVLNode* t)
{
	if (t != NULL)
	{
		dispose(t->left);
		dispose(t->right);
		free(t);
	}
}

/*
remove a node in the tree
*/
AVLNode* delete(int e, AVLNode* t)
{
	printf("Sorry; Delete is unimplemented; %d remains\n", e);
	return t;
}
/*
struct AVLNode* deleteNode(struct AVLNode* root, int key)
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
struct AVLNode *temp = root->left ? root->left : root->right;

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
struct AVLNode* temp = minValueNode(root->right);

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
