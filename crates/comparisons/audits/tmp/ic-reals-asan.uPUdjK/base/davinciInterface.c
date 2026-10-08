/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <stdio.h>
#include "real.h"
#include "real-impl.h"
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>

/*
 * This file defines the functions for talking to the graph visualization
 * tool daVinci. The collection of functions are divided into two groups.
 *
 * The first part of this file deals with sending graph updates to daVinci. 
 * Updates consist of instructions to create new nodes or edges. 
 *
 * The second part of the file deals with reading and processing daVinci
 * answers.
 *
 * DaVinci requires that all nodes are assigned unique identifiers. 
 * Each heap object is assigned an identifier (an integer) when created
 * Unlike the address of the object, this is invariant under
 * copying garbage collection.
 *
 * Edges connect nodes to child nodes.  As with nodes, daVinci requires
 * that we assign a unique identifier (a string) to each edge. This
 * is formed from the id of the source of the edge and the index of
 * the child.
 *
 * A sequence of calls to the functions to create new nodes and
 * edges must be bracketted between calls to beginGraphUpdate() and
 * endGraphUpdate(). Calls to create new nodes and edges can appear in
 * any order between the begin and end.
 *
 * According to the daVinci documentation, one can send a list
 * of "mixed_updates" with new_node and new_edge commands in any
 * order. However, there is a bug and mixed updates don't work. Node and
 * edge updates must be in separate lists. The code below gets around
 * this bug.
 */

char davBuf[2 * 1024];
char *davPtr;
int makeNodeUpdateList = 0;

/*
 * The ``abstact machine'' has four states. RUNNING and STOPPED apply when
 * the stack is non-empty. When the stack runs out and we are about to
 * return to the caller, the machine state WAITING, that way we can still
 * interact with daVinci before passing control back.
 */
#define RUNNING 1
#define STOPPED 2
#define WAITING 3
#define FINISHED 4

int machineState = STOPPED;

int repliesExpected = 1; /* from the outset we expect an ok from davinci */

#define NODE_LIST 1
#define EDGE_LIST 2

int toDavFds[2];
int fromDavFds[2];
FILE *davReadFP, *davWriteFP;

void setMachineState(int);

/*
 * Communication with daVinci is through a bounded buffer. I would prefer
 * to use stdio(3) buffered IO since I want to work with lines (terminated
 * with newlines) suggesting fgets(3), but I want non-blocking IO,
 * which seems not possible with the stdio(3) functions.
 */
struct {
	char start[BUFSIZ];
	char *current;
	int count;
} daBuf;
 
/*
 * This initializes the interface to daVinci. We create a couple of pipes
 * spawn the daVinci process, and then send some initialization parameters.
 */
initDaVinci()
{
	int pid;
	char buf[1024];

	/*
	 * We need two pipes to talk with daVinci. Info flows both ways.
	 */
	if (pipe(toDavFds) != 0)
		Error(FATAL, E_SYS, "initDavinci", "can't open to daVinci pipe\n");

	if (pipe(fromDavFds) != 0)
		Error(FATAL, E_SYS, "initDavinci", "can't open from daVinci pipe\n");

	switch (pid = fork()) {
	case -1 :	/* parent error */
		Error(FATAL, E_SYS, "initDavinci", "can't fork\n");
		break;

	case 0 : 	/* child */
		if (close(toDavFds[1]) == -1)
			Error(FATAL, E_SYS, "initDavinci (child)", "close toDav failed\n");

		if (dup2(toDavFds[0], fileno(stdin)) == -1)
			Error(FATAL, E_SYS, "initDavinci (child)", "dup2 stdin failed\n");

		if (close(fromDavFds[0]) == -1)
			Error(FATAL, E_SYS, "initDavinci (child)", "close fromDav failed\n");

		if (dup2(fromDavFds[1], fileno(stdout)) == -1)
			Error(FATAL, E_SYS, "initDavinci (child)", "dup2 stdout failed\n");

		sprintf(buf, "DAVINCI_ICONDIR=%s/icons", REALDIR);
		putenv(buf);

		if (execlp("daVinci", "daVinci", "-pipe", (char *)0) == -1)
			Error(FATAL, E_SYS, "initDavinci (child)",
										"execlp daVinci -pipe failed\n");
		break;

	default :	/* parent */
		if (close(fromDavFds[1]) == -1)
			Error(FATAL, E_SYS, "initDavinci (parent)", "close fromDav failed\n");

		if ((davReadFP = fdopen(fromDavFds[0], "r")) == NULL)
			Error(FATAL, E_SYS, "initDavinci (parent)", "read fdopen failed\n");

		if (close(toDavFds[0]) == -1)
			Error(FATAL, E_SYS, "initDavinci (parent)", "close toDav failed\n");

		if ((davWriteFP = fdopen(toDavFds[1], "w")) == NULL)
			Error(FATAL, E_SYS, "initDavinci (parent)",
											"write fdopen failed\n");

		setlinebuf(davWriteFP);
		setlinebuf(davReadFP); /* useless */
		break;
	}

	/*
	 * Now set up the bounded buffer for receiving answers from daVinci.
	 */
	daBuf.count = 0;
	daBuf.current = daBuf.start;

	/*
	 * Get the initial "ok" (though it doesn't seem necessary)
	 */
	handleDaVinciMessages(BLOCK);

	fprintf(davWriteFP, "menu(layout(orientation(left_right)))\n");
	fprintf(davWriteFP, "set(font_size(8))\n");
	fprintf(davWriteFP, "set(keep_nodes_at_levels(false))\n");
	fprintf(davWriteFP, "set(layout_accuracy(1))\n");
	fprintf(davWriteFP, "set(gap_height(10))\n");
	fprintf(davWriteFP, "set(gap_width(10))\n");
	fprintf(davWriteFP, "app_menu(create_icons([");
	fprintf(davWriteFP, "icon_entry(\"stop-program\",\"stop.xbm\",\"Stop\"),");
	fprintf(davWriteFP, "icon_entry(\"run-program\",\"go.xbm\",\"Run\"),");
	fprintf(davWriteFP, "icon_entry(\"step-program\",\"step.xbm\",\"Single step\"),");
	fprintf(davWriteFP, "icon_entry(\"continue-program\",\"continue.xbm\",\"Continue\"),");
	fprintf(davWriteFP, "icon_entry(\"collect-garbage\",\"collect.xbm\",\"Garbage Collection\")]))\n");

	/*
	 * The number of replies expected should be the same as the number
	 * of newlines transmitted. Perhaps the code should reflect that.
	 */
	repliesExpected += 7;
	setMachineState(STOPPED);
}

static int updateState;

/*
 * Calls to add and delete edges must be bracketed between calls to
 * beginGraphUpdate() and endGraphUpdate(). This serves also to get around
 * a bug in daVinci.
 */
beginGraphUpdate()
{
	davPtr = davBuf + sprintf(davBuf, "graph(update([");
	updateState = NODE_LIST;
}

endGraphUpdate()
{
	if (*(davPtr - 1) == ',')
		davPtr--;
	if (updateState == NODE_LIST)
		davPtr += sprintf(davPtr, "],[");
	davPtr += sprintf(davPtr, "]))\n");
	repliesExpected++;
	fputs(davBuf, davWriteFP);
/*
	fputs(davBuf, stderr);
	fflush(stderr);
*/
	/* fflush(davWriteFP); */
}

/*
 * This assigns a couple to each type of object in the heap. Yes the case
 * statement is slow, but the moment is doesn't matter.
 */
char *
typeToColor(unsigned type)
{
	switch (type) {
		case ALT :
			return "pink";
		case VECTOR :
			return "red";
		case MATX :
			return "red";
		case TENXY :
			return "red";
		case SIGNX :
			return "blue";
		case DIGSX :
			return "cyan";
		case CLOSURE :
			return "green";
		case BOOLX :
			return "orange";
		case BOOLXY :
			return "orange";
		case PREDX :
			return "yellow";
		default :
			Error(FATAL, E_INT, "typeToColor", "bad type: %d", type);
			break;
	}
}

/*
 * This assigns a shape (box, circle, rhombus etc) to render each heap object
 * with.
 */
char *
typeToShape(unsigned type)
{
	switch (type) {
		case ALT :
			return "circle";
		case VECTOR :
			return "box";
		case MATX :
			return "box";
		case TENXY :
			return "box";
		case SIGNX :
			return "circle";
		case DIGSX :
			return "circle";
		case CLOSURE :
			return "box";
		case BOOLX :
			return "box";
		case BOOLXY :
			return "box";
		case PREDX :
			return "box";
		default :
			Error(FATAL, E_INT, "typeToColor", "bad type: %d", type);
			break;
	}
}

/*
 * This assigns a string label to each type of object in the heap. Not used
 * at present.
 */
char *
typeToLabel(unsigned type)
{
	switch (type) {
		case ALT :
			return "A";
		case VECTOR :
			return "V";
		case MATX :
			return "M";
		case TENXY :
			return "T";
		case SIGNX :
			return "S";
		case DIGSX :
			return "D";
		case CLOSURE :
			return "C";
		case BOOLX :
			return "U";
		case BOOLXY :
			return "N";
		case PREDX :
			return "P";
		default :
			Error(FATAL, E_INT, "typeToLabel", "bad type: %d", type);
			break;
	}
}

newNode(Generic *node, ObjType nodetype)
{
	if (updateState != NODE_LIST) {
		endGraphUpdate();
		beginGraphUpdate();
	}

	davPtr += sprintf(davPtr, "new_node(");
	davPtr += sprintf(davPtr, "\"n%d\"", node->tag.nodeId);
	davPtr += sprintf(davPtr, ",\"%s\"", typeToString(nodetype));
	davPtr += sprintf(davPtr, ",[");
/*
	davPtr += sprintf(davPtr, "a(\"_GO\",\"icon\")");
	davPtr += sprintf(davPtr, ",a(\"ICONFILE\",\"node.xbm\")");
	davPtr += sprintf(davPtr, ",a(\"BORDER\",\"none\")");
	davPtr += sprintf(davPtr, "a(\"OBJECT\",\"%s\")", typeToLabel(nodetype));
*/
	davPtr += sprintf(davPtr, "a(\"_GO\",\"%s\")", typeToShape(nodetype));
	davPtr += sprintf(davPtr, ",a(\"OBJECT\",\" \")");
/*
	davPtr += sprintf(davPtr, ",a(\"OBJECT\",\"%d\")", node->tag.nodeId);
*/
	davPtr += sprintf(davPtr, ",a(\"COLOR\",\"%s\")", typeToColor(nodetype));
	davPtr += sprintf(davPtr, ",m([");
	davPtr += sprintf(davPtr, "menu_entry(\"set-break\",\"Set break\")");
	davPtr += sprintf(davPtr, ",menu_entry(\"clear-break\",\"Clear break\")");
	davPtr += sprintf(davPtr, ",menu_entry(\"show-contents\",\"Show contents\")");
	davPtr += sprintf(davPtr, "])");
	davPtr += sprintf(davPtr, "]),");
}

/*
 * This connects node1 to node2 where node2 is childIdx is the index amongst
 * all the children of node1. The index is needed since, node1 maybe 
 * connected to node2 more than once and we need to distinguish the
 * edges.
 */
newEdgeToChildN(Generic *node1, Generic *node2, int childIdx)
{
	if (updateState == NODE_LIST) {
		if (*(davPtr - 1) == ',')
			davPtr--;
		davPtr += sprintf(davPtr, "],[");
		updateState = EDGE_LIST;
	}

	davPtr += sprintf(davPtr, "new_edge(");
	davPtr += sprintf(davPtr, "\"e%d.%d.%d\"",
				node1->tag.nodeId, node2->tag.nodeId, childIdx);
/*
	printf("new e%d.%d.%d\n", node1->tag.nodeId, node2->tag.nodeId, childIdx);
*/
	davPtr += sprintf(davPtr, ",\"edge\"");
	davPtr += sprintf(davPtr, ",[]");
	davPtr += sprintf(davPtr, ",\"n%d\"", node1->tag.nodeId);
	davPtr += sprintf(davPtr, ",\"n%d\"),", node2->tag.nodeId);
}

/*
 * This is exactly the same as the above, only the edge is drawn
 * double and without an arrow. This is used to connect two node which
 * denote the same real value.
 */
drawEqEdge(Generic *node1, Generic *node2)
{
	if (updateState == NODE_LIST) {
		if (*(davPtr - 1) == ',')
			davPtr--;
		davPtr += sprintf(davPtr, "],[");
		updateState = EDGE_LIST;
	}

	davPtr += sprintf(davPtr, "new_edge(");
	davPtr += sprintf(davPtr, "\"e%d.%d.eq\"",
				node1->tag.nodeId, node2->tag.nodeId);
/*
	printf("new eq e%d.%d.eq\n", node1->tag.nodeId, node2->tag.nodeId);
*/
	davPtr += sprintf(davPtr, ",\"edge\"");
	davPtr += sprintf(davPtr, ",[");
	davPtr += sprintf(davPtr, "a(\"_DIR\",\"none\")"); 
	davPtr += sprintf(davPtr, ",a(\"EDGEPATTERN\",\"double\")"); 
	davPtr += sprintf(davPtr, "],\"n%d\"", node1->tag.nodeId);
	davPtr += sprintf(davPtr, ",\"n%d\"),", node2->tag.nodeId);
}


highlightEdge(Generic *node1, Generic *node2, int childIdx)
{
	davPtr = davBuf + sprintf(davBuf, "graph(change_attr([");
	davPtr += sprintf(davPtr, "edge(");
	davPtr += sprintf(davPtr, "\"e%d.%d.%d\"",
				node1->tag.nodeId, node2->tag.nodeId, childIdx);
/*
	printf("e%d.%d.%d\n", node1->tag.nodeId, node2->tag.nodeId, childIdx);
*/
	davPtr += sprintf(davPtr, ",[");
	davPtr += sprintf(davPtr, "a(\"EDGECOLOR\",\"red\")"); 
	davPtr += sprintf(davPtr, ",a(\"EDGEPATTERN\",\"dashed\")"); 
	davPtr += sprintf(davPtr, "])]))\n");
	repliesExpected++;
	fputs(davBuf, davWriteFP);
	/* fflush(davWriteFP); */
}

unhighlightEdge(Generic *node1, Generic *node2, int childIdx)
{
	davPtr = davBuf + sprintf(davBuf, "graph(change_attr([");
	davPtr += sprintf(davPtr, "edge(");
	davPtr += sprintf(davPtr, "\"e%d.%d.%d\"",
				node1->tag.nodeId, node2->tag.nodeId, childIdx);
	davPtr += sprintf(davPtr, ",[");
	davPtr += sprintf(davPtr, "a(\"EDGECOLOR\",\"black\")"); 
	davPtr += sprintf(davPtr, ",a(\"EDGEPATTERN\",\"solid\")"); 
	davPtr += sprintf(davPtr, "])]))\n");
	repliesExpected++;
	fputs(davBuf, davWriteFP);
	/* fflush(davWriteFP); */
}

highlightNode(Generic *node)
{
	davPtr = davBuf + sprintf(davBuf, "graph(change_attr([");
	davPtr += sprintf(davPtr, "node(");
	davPtr += sprintf(davPtr, "\"n%d\"", node->tag.nodeId);
	davPtr += sprintf(davPtr, ",[");
	davPtr += sprintf(davPtr, "a(\"BORDER\",\"double\")"); 
	davPtr += sprintf(davPtr, "])]))\n");
	repliesExpected++;
	fputs(davBuf, davWriteFP);
	/* fflush(davWriteFP); */
}

unhighlightNode(Generic *node)
{
	davPtr = davBuf + sprintf(davBuf, "graph(change_attr([");
	davPtr += sprintf(davPtr, "node(");
	davPtr += sprintf(davPtr, "\"n%d\"", node->tag.nodeId);
	davPtr += sprintf(davPtr, ",[");
	davPtr += sprintf(davPtr, "a(\"BORDER\",\"single\")"); 
	davPtr += sprintf(davPtr, "])]))\n");
	repliesExpected++;
	fputs(davBuf, davWriteFP);
	/* fflush(davWriteFP); */
}

/*
 * Some convenient abbreviations of the the newEdge function
 */
newEdgeToOnlyChild(Generic *node1, Generic *node2)
{
	newEdgeToChildN(node1, node2, 0);
}

newEdgeToXChild(Generic *node1, Generic *node2)
{
	newEdgeToChildN(node1, node2, 0);
}

newEdgeToYChild(Generic *node1, Generic *node2)
{
	newEdgeToChildN(node1, node2, 1);
}

deleteEdgeToChildN(Generic *node1, Generic *node2, int childIdx)
{
	if (updateState == NODE_LIST) {
		if (*(davPtr - 1) == ',')
			davPtr--;
		davPtr += sprintf(davPtr, "],[");
		updateState = EDGE_LIST;
	}

	davPtr += sprintf(davPtr, "delete_edge(");
	davPtr += sprintf(davPtr, "\"e%d.%d.%d\"",
			node1->tag.nodeId, node2->tag.nodeId, childIdx);
	davPtr += sprintf(davPtr, "),");
}

/*
 * More legacy abbreviations.
 */
deleteOnlyEdge(Generic *node1, Generic *node2)
{
	deleteEdgeToChildN(node1, node2, 0);
}

deleteEdgeToXChild(Generic *node1, Generic *node2)
{
	deleteEdgeToChildN(node1, node2, 0);
}

deleteEdgeToYChild(Generic *node1, Generic *node2)
{
	deleteEdgeToChildN(node1, node2, 1);
}

void
setMachineState(int state)
{
	if (state == STOPPED) {
		machineState = STOPPED;
		fprintf(davWriteFP, "app_menu(activate_icons([\"run-program\",\"step-program\"]))\n");
		/* fflush(davWriteFP); */
		repliesExpected++;
		return;
	}
	if (state == RUNNING) {
		machineState = RUNNING;
		fprintf(davWriteFP, "app_menu(activate_icons([\"stop-program\"]))\n");
		/* fflush(davWriteFP); */
		repliesExpected++;
		return;
	}
	if (state == WAITING) {
		machineState = WAITING;
		fprintf(davWriteFP, "app_menu(activate_icons([\"continue-program\"]))\n");
		/* fflush(davWriteFP); */
		repliesExpected++;
		return;
	}
	if (state == FINISHED) {
		machineState = FINISHED;
		fprintf(davWriteFP, "app_menu(activate_icons([]))\n");
		/* fflush(davWriteFP); */
		repliesExpected++;
		return;
	}
}

void
singleStep()
{
	void (*f)();

	if (machineState == STOPPED && sp >= stack) {
		if (TRACE)
			dumpTopOfStack();
		unhighlightTOS();
		f = (void (*)()) POP;
		(*f)();
	}
}

/*
 * I include both of these for portability. Linux needs only the first
 * while Solaris seems to need both
 */
#include <string.h>
#include <strings.h>

/*
 * For each answer (a string) we attach an action (a function). This
 * takes a string as an argument which is the remaining unparsed string
 * (whether or not there is anything left). These functions return a
 * pointer to the next character beyond the answer.
 */
typedef struct {
	char *string;
	char *(*action)(char *);
} Token;

/*
 * There are a number of answers/messages from daVinci for which we are
 * not interested in in which case, the following is their action.
 */
char *
doNothing(char *p)
{
	return p;
}

/*
 * Action for "ok" message.
 */
char *
doOK(char *p)
{
	repliesExpected--;
	return p;
}

/*
 * Action for "exit" message.
 */
char *
doExit(char *p)
{
	fprintf(stderr, "\n");
	exit(0);
}

/*
 * This is for messages (answers) from daVinci which we don't care about
 * but which include an argument delimited by '(' and ')'. So we just go
 * scan for ')' and return a pointer to the next character. The assumption
 * here is that there are no nested parentheses in answers (ie in node and
 * edge identifiers) which is reasonable since we are the ones choosing
 * the identifiers. The + 2 skips the closing bracket and the newline.
 */
char *
doNothingWithArg(char *p)
{
/*
	char *q;

	q = index(p, ')');
	*q = '\0';
	fprintf(stderr, "%s", p + 1);
	return q + 2;
*/
	return index(p, ')') + 2;
}

char *
iconSelection(char *p)
{
	char *q;

	/*
	 * p is at the opening '"', we look for the closing ')'
	 */
	p++;
	q = index(p, '"');
	*q = '\0';

	if (strcmp(p, "stop-program") == 0)
		setMachineState(STOPPED);
	else {
		if (strcmp(p, "run-program") == 0)
			setMachineState(RUNNING);
		else {
			if (strcmp(p, "step-program") == 0) {
				singleStep();
			}
			else
				if (strcmp(p, "continue-program") == 0)
					setMachineState(FINISHED);
				else
					fprintf(stderr, "bad icon selection: %s\n", p+1);
		}
	}

	return q + 3;
}

/* 
 * Activated when we get a "popup-selection-node" message. This happens when
 * the user clicks on a menu entry attached to the node popup. The menu
 * includes entries for setting and clearing breakpoints and for displaying
 * the contents of the object in the heap. Only the last, "show-contents",
 * is implemented.
 */
char *
popupSelectionNode(char *p)
{
	char *q;
	unsigned nodeId;
	Generic *addr, *mapNodeIdToHeapCell(int);

	/*
	 * p is at the '"', and we look for the closing '"'.
	 */
	q = index(p + 1, '"');
	*q = '\0';

	/*
	 * the first argument should be a node id which is a string enclosed
	 * in '"' with prefix 'n' followed by a number
	 * so we skip the quotes and the 'n'.
	 */
	sscanf(p + 2, "%d", (unsigned *) &nodeId);

	/*
	 * q is at the null and next there is a ',' and '"' which we skip
	 */
	p = q + 3;
	
	/*
	 * The second argument should be a string enclosed in '"' which should
	 * be the menu entry
	 */
	q = index(p, '"');
	*q = '\0';
	if (strcmp(p, "show-contents") == 0)
		dumpCell((void *) mapNodeIdToHeapCell(nodeId));
	else
		fprintf(stderr, "unknown menu entry: %s\n", q);

	/*
	 * q is at the null, we skip the closing ')' and newline and return.
	 */
	return q + 3;
}

/*
 * Called when we get a "communication-error" message from daVinci.
 * Just write the message and exit.
 */
char *
communicationError(char *p)
{
	char *q;

	/*
	 * The first character should be '"' which we skip and the
	 * scan for a closing bracket. The character preceeding the
	 * closing bracket should also be '"' which we clobber with a
	 * null char and then return q+2 to skip the closing bracket
	 * and the newline.
	 * For now, a communication error exits the program.
	 */
	q = index(p + 1, ')');
	*(q - 1) = '\0';     
	Error(FATAL, E_INT, "daVinci interface", "communication error %s", (p + 1));
	return q + 2;
}

/*
 * The following is the list of possible answers provided by daVinci. The list
 * is sorted for a binary search.
 */
Token tokens[] = {
	{"browser_answer", doNothingWithArg},			/* (string,string) */
	{"close", doNothing},
	{"close_window", doNothingWithArg},				/* (window_id) */
	{"communication_error", communicationError},	/* (string) */
	{"context", doNothingWithArg},					/* (context_id) */
	{"context_window", doNothingWithArg},			/* (context_id,window_id) */
	{"create_edge", doNothingWithArg},				/* (node_id,node_id) */
	{"create_node", doNothing},
	{"create_node_and_edge", doNothingWithArg},		/* (node_id) */
	{"disconnect", doExit},
	{"drop_node", doNothingWithArg},	/* (node_id,context_id,window_id,node_id) */
	{"edge_double_click", doNothing},
	{"edge_selection_label", doNothingWithArg},		/* (edge_id) */
	{"edge_selection_labels", doNothingWithArg},	/* (node_id,node_id) */
	{"icon_selection", iconSelection},				/* (icon_id) */
	{"menu_selection", doNothingWithArg},			/* (menu_id) */
	{"node_double_click", doNothing},
	{"node_selections_labels", doNothingWithArg},	/* (node_ids) */
	{"ok", doOK},
	{"open_window", doNothing},
	{"popup_selection_edge", doNothingWithArg},		/* (edge_id,menu_id) */
	{"popup_selection_node", popupSelectionNode},	/* (node_id,menu_id) */
	{"quit", doExit},
	{"tcl_answer", doNothingWithArg}				/* (string) */
};

static int
compare(const void *t1, const void *t2)
{
	return strcmp(((Token *)t1)->string, ((Token *)t2)->string);
}

/*
 * This retieves the next message from the daVinci buffer. A message is complete
 * when it ends in a newline. This copies the message into the given line
 * buffer and terminates it with a null. The buffer is assumed to be
 * big enough to receive the line. The function returns TRUE if the newline
 * is found. If no newline is found, the function returns FALSE and the
 * daVinci buffer is unaffected.
 */
int
getNextAnswer(char *line)
{
	char *current;
	int count;
	char c;
 
	count = daBuf.count;
	current = daBuf.current;
	while (count > 0) {
		c = *current;
		count--;
		*line++ = *current++;
		if (current == daBuf.start + BUFSIZ)
			current = daBuf.start;
		if (c == '\n') {
			*line = '\0';
			daBuf.count = count;
			daBuf.current = current;
			return TRUE;
		}
	}
	if (daBuf.count == BUFSIZ)
		Error(FATAL, E_INT, "getNextAnswer",
			"buffer full but no complete answers");
 
	return FALSE;
}

/*
 * As the name suggests, this function handles the messages from daVinci.
 * We read from davReadFP (into a fixed size buffer) and then parse and handle
 * the different messages. It is possible (and likely) that in some cases there
 * will be more than one message in the buffer.
 */
readAndProcessDaVinciMessages()
{
	char *p, line[BUFSIZ];
	Token key, *tokPtr;
	int n;
	char *next;
	int count;
	int size;
/*
	char *strsep(char **, char *);
*/
 
	/*
	 * First we try to read what we can from the pipe into the space left
	 * in the buffer.
	 */
	if (daBuf.count < BUFSIZ) {
		next = ((daBuf.current - daBuf.start + daBuf.count) % BUFSIZ)
													+ daBuf.start;
		if (next >= daBuf.current)
			size = daBuf.start + BUFSIZ - next;
		else
			size = daBuf.current - next;
		n = read(fileno(davReadFP), next, size);

		if (n == -1)
			Error(FATAL, E_SYS, "readAndProcessDaVinciMessages", "read failed");
		if (n == 0)
			Error(FATAL, E_INT, "readAndProcessDaVinciMessages",
										"unexpected EOF");
		daBuf.count += n;
	}

	/*
	 * Now we go through the buffer and process all the answers and
	 * messages sent from daVinci.
	 */
	while (getNextAnswer(line)) {
		for (p = line; (key.string = strsep(&p, "\n(")) != NULL;) {
			if (*key.string != '\0') {
				tokPtr = (Token *) bsearch(&key, tokens,
						sizeof(tokens) / sizeof(Token), sizeof(Token), compare);
	
				if (tokPtr == NULL)
					Error(FATAL, E_INT, "readAndProcessDaVinciMessages",
								"bad answer: %s", key.string);
				/*
				 * Now we activate the function associated with the 
				 * message received
				 */
				p = (*(tokPtr->action))(p);
			}
		}
	}
}

/*
 * This is the function actually called to read and process daVinci messages.
 * If the parameter is true, it will block until data is available. Otherwise
 * it polls (via select(2)).
 */
handleDaVinciMessages(int block)
{
	fd_set rfds;
	struct timeval tv;
	int retval;

	/* Watch davinci input fd to see when it has input. */

	do {
		FD_ZERO(&rfds);
		FD_SET(fileno(davReadFP), &rfds);
	
		if (block || repliesExpected > 0)
			retval = select(fileno(davReadFP) + 1, &rfds, NULL, NULL, NULL);
		else {
			/*
			 * Set timeout to 0, so we are essentially polling
			 */
			tv.tv_sec = 0;
			tv.tv_usec = 0;
			retval = select(fileno(davReadFP) + 1, &rfds, NULL, NULL, &tv);
		}
	
		switch (retval) {
		case -1 :
			Error(FATAL, E_SYS, "", "select failed");
			break;
	
		case 0 :
			break;
			
		default :
			if (FD_ISSET(fileno(davReadFP), &rfds)) {
				readAndProcessDaVinciMessages();
			}
			break;
		}
	} while (repliesExpected > 0);
}

/*
 * The abstract machine can be controlled via the icons on the left side of
 * the daVinci window. This is the function used to run the stack when 
 * daVinci is compiled in. 
 */
void
runStackViaDaVinci()
{
	void (*f)();

	setMachineState(STOPPED);

	while (sp >= stack) {
		if (machineState == RUNNING)
			handleDaVinciMessages(!BLOCK);
		else
			handleDaVinciMessages(BLOCK);
		if (machineState == RUNNING) {
			if (TRACE)
				dumpTopOfStack();
			unhighlightTOS();
        	f = (void (*)()) POP;
        	(*f)();
		}
	}

	setMachineState(WAITING);
	while (machineState == WAITING)
		handleDaVinciMessages(BLOCK);
}
