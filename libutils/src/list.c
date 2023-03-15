/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <utils/list.h>
#include <utils/attribute.h>

typedef struct list_node node_t;

int list_init(list_t *l)
{
    assert(l != NULL);
    l->head = NULL;
    return 0;
}

static node_t *make_node(void *data)
{
    node_t *n = malloc(sizeof(*n));
    if (n != NULL) {
        n->data = data;
        n->next = NULL;
    }
    return n;
}

int list_prepend(list_t *l, void *data)
{
    node_t *n = make_node(data);
    if (n == NULL) {
        return -1;
    }
    return list_prepend_node(l, n);
}

int list_append(list_t *l, void *data)
{
    node_t *n = make_node(data);
    if (n == NULL) {
        return -1;
    }
    return list_append_node(l, n);
}

bool list_is_empty(list_t *l)
{
    assert(l != NULL);
    return l->head == NULL;
}

bool list_exists(list_t *l, void *data, int(*cmp)(void *, void *))
{
    assert(l != NULL);
    for (node_t *n = l->head; n != NULL; n = n->next) {
        if (!cmp(n->data, data)) {
            return true;
        }
    }
    return false;
}

int list_length(list_t *l)
{
    assert(l != NULL);
    int i = 0;
    for (node_t *n = l->head; n != NULL; n = n->next) {
        i++;
    }
    return i;
}

int list_index(list_t *l, void *data, int(*cmp)(void *, void *))
{
    assert(l != NULL);
    int i = 0;
    for (node_t *n = l->head; n != NULL; n = n->next, i++) {
        if (!cmp(n->data, data)) {
            return i;
        }
    }
    return -1;
}

int list_foreach(list_t *l, int(*action)(void *, void *), void *token)
{
    assert(l != NULL);
    for (node_t *n = l->head; n != NULL; n = n->next) {
        int res = action(n->data, token);
        if (res != 0) {
            return res;
        }
    }
    return 0;
}

static int remove(list_t *l, void *data, int (*cmp)(void *, void *),
                  bool should_free)
{
    assert(l != NULL);
    for (node_t *prev = NULL, *n = l->head; n != NULL; prev = n, n = n->next) {
        if (!cmp(n->data, data)) {
            if (prev == NULL) {
                /* Removing the list head. */
                l->head = n->next;
            } else {
                prev->next = n->next;
            }
            if (should_free) {
                free(n);
            }
            return 0;
        }
    }
    return -1;
}

int list_remove(list_t *l, void *data, int(*cmp)(void *, void *))
{
    return remove(l, data, cmp, true);
}

int list_remove_all(list_t *l)
{
    assert(l != NULL);
    for (node_t *n = l->head; n != NULL;) {
        node_t *temp = n->next;
        free(n);
        n = temp;
    }
    l->head = NULL;
    return 0;
}

int list_destroy(UNUSED list_t *l)
{
    /* Nothing required. */
    return 0;
}

int list_prepend_node(list_t *l, node_t *node)
{
    assert(l != NULL);
    assert(node != NULL);
    node->next = l->head;
    l->head = node;
    return 0;
}

int list_append_node(list_t *l, node_t *node)
{
    assert(l != NULL);
    assert(node != NULL);
    node->next = NULL;
    if (l->head == NULL) {
        l->head = node;
    } else {
        node_t *end;
        for (end = l->head; end->next != NULL; end = end->next);
        end->next = node;
    }
    return 0;
}

int list_remove_node(list_t *l, void *data, int(*cmp)(void *, void *))
{
    return remove(l, data, cmp, false);
}

int list_remove_all_nodes(list_t *l)
{
    assert(l != NULL);
    l->head = NULL;
    return 0;
}

void *list_peek(list_t *l)
{
    assert(l != NULL);
    return (l->head->data);
}

void *list_peek_idx(list_t *l, int idx)
{
    assert(l != NULL);

    int i = 0;
    node_t *node = l->head;

    for ( ; node != NULL; node = node->next, i++) {
        if (i == idx) {
            break;
        }
    }

    return (node != NULL ? node->data : NULL);
}

void *list_pop(list_t *l)
{
    assert(l != NULL);
    void *data = NULL;
    node_t *node = l->head;

    if (node != NULL) {
        l->head = node->next;
        node->next = NULL;

        data = node->data;
        free(node);
    }

    return data;
}

void *list_pop_idx(list_t *l, int idx)
{
    assert(l != NULL);

    void *data = NULL;
    node_t *node = l->head;
    node_t *prev = node;
    int i = 0;

    for (; node != NULL; prev = node, node = node->next, i++) {
        if (i == idx) {
            if (prev == NULL) {
                /* Removing the list head. */
                l->head = node->next;
                node->next = NULL;
            } else {
                prev->next = node->next;
                node->next = NULL;
            }
            break;
        }
    }

    if (node != NULL) {
        data = node->data;
        free(node);
    }

    return data;
}

void *list_tail(list_t *l)
{
    void *data = NULL;
    node_t *node = l->head;
    node_t *prev = node;

    /* Wind to the end of the list */
    for (; node->next != NULL; prev = node, node = node->next);

    /* Remove last item */
    prev->next = NULL;

    if (node != NULL) {
        data = node->data;
        free(node);
    }

    return data;
}


/* Sort the list and return a pointer to the tail node */
static node_t *qsort_list(node_t **head, int (*cmp)(const void *, const void *)) 
{
    /* Sanity check that list exists */
    if (head == NULL) {
        return NULL;
    }

    /* Basic cases: empty list and single node */
    if ((*head) == NULL) {
        return NULL;
    }
    if ((*head)->next == NULL) {
        return (*head);
    }

    /* Partition the list into 3 sublists using first node (== *head) as pivot. 
     * 
     * The left_front/left_right sublist is for nodes, for which the "cmp" 
     * function returns <0, and the right_front/right_tail sublist is for nodes, 
     * for which the "cmp" function returns >0. 
     * 
     * The pivot/pivot_front list is for nodes, for which the "cmp"
     * function returned 0, meaning they are equal. Equal nodes are
     * prepended before the pivot node. */

    /* First node is the pivot */
    node_t *pivot       = (*head);
    node_t *pivot_front = pivot;
    node_t *left_front  = NULL;
    node_t *left_tail   = NULL;
    node_t *right_front = NULL;
    node_t *right_tail  = NULL;

    /* Start from the node after pivot */
    node_t *curr_node   = (*head)->next;

    /* Pivot's next field must be reset, otherwise
     * the list will eventually link to itself. */
    pivot->next = NULL;

    while (curr_node != NULL) {

        node_t *next_node = curr_node->next;
        int rc = cmp((const void *)curr_node->data, (const void *)pivot->data);

        /* Prepending to the list is faster than appening */
        if (rc == 0) {
            curr_node->next = pivot_front;
            pivot_front = curr_node;
        } else if (rc < 0) {
            curr_node->next = left_front;
            left_front = curr_node;
        } else {
            curr_node->next = right_front;
            right_front = curr_node;
        }

        curr_node = next_node;
    }

    /* Sort split lists recursively if needed.
     * Then, join the lists:
     * {left_front, left_tail} + {pivot_front, pivot_tail} + {right_front, right_tail} 
     */
    if (left_front != NULL) {

        /* Sort left sublist recursively */
        left_tail = qsort_list(&left_front, cmp);
        
        /* Join left to pivot */
        left_tail->next = pivot_front;

        /* Update original lists' head */
        (*head) = left_front;
    } else {
        /* Left sublist was empty. 
         * Update original lists' head  */
        (*head) = pivot_front;
    }

    if (right_front != NULL) {

        /* Sort right sublist recursively */
        right_tail = qsort_list(&right_front, cmp);

        /* Join pivot to right */
        pivot->next = right_front;
    } else {
        /* Right sublist was empty.
         * Tail node is the pivot */
        right_tail = pivot;
    }

    /* Terminate the joined list */
    right_tail->next = NULL;

    /* Return the tail of the new list */
    return right_tail;
}


void list_qsort(list_t *l, int (*cmp)(const void *, const void *)) 
{
    assert(l != NULL);

    node_t **head = &(l->head);
    qsort_list(head, cmp);
}
