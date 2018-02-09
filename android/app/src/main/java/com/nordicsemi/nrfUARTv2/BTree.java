package com.nordicsemi.nrfUARTv2;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by hikari on 2018/1/6.
 */


public class BTree {
    public static class BNode {
        int data;
        BNode left;
        BNode right;
        BNode(int data)
        {
            this.data=data;
        }
    }
    private List<Integer> mPostOrd = new ArrayList<Integer>();
    private BNode mRootNode;
    public byte[] getPostOrd(){
        mPostOrd.clear();
        postOrder(mRootNode);
        byte [] ret = new byte[mPostOrd.size()];
        for (int i=0;i<mPostOrd.size();i++){
            ret[i] = mPostOrd.get(i).byteValue();
        }
        return ret;
    }
    public BTree(){
        mRootNode = new BNode(0);
    }
    public BNode getRoot() {
        return mRootNode;
    }
    public void addRootNode(int data){
        mRootNode.data = data;
    }
    private boolean getDir(int add) {
        if( ((add) & (0x10<<((add)&0x03))) > 0 )
            return true;
        else
            return false;
    }
    private Boolean hasNextPack(int add) {
        if((add&0x03) != 0)
            return true;
        else
            return false;
    }
    public boolean refreshNode(int add,int left,int right) {
        int add_right = 0,add_left=0;
        add_left = ((add&0xf0)<<1) + (add&0x0f)+1;
        add_right = add_left + 0x20;
        BNode pNode = mRootNode;
        while(hasNextPack(add)) {
            if(getDir(add)) {//search right
                if(pNode.right == null)
                    return false;
                pNode = pNode.right;
            }
            else {//search left
                if(pNode.left == null)
                    return false;
                pNode = pNode.left;
            }
            add -= 1;

        }
        if(left>0) {
            if(pNode.left == null) {
                pNode.left = new BNode(add_left);
            }
        }
        else {
            if(pNode.left != null) {
                pNode.left = deleteTree(pNode.left);
            }

        }

        if(right>0) {
            if(pNode.right == null) {
                pNode.right = new BNode(add_right);
            }
        }
        else {
            if(pNode.right != null) {
                pNode.right = deleteTree(pNode.right);
            }

        }
        return true;
    }

    private BNode deleteTree(BNode root) {
        if(null == root) {
            return null;
        }
        root.left  =  deleteTree(root.left);
        root.right =  deleteTree(root.right);
        root = null;
        return root;
    }

    private void postOrder(BNode root) {
        if(root !=  null) {
            postOrder(root.left);
            postOrder(root.right);
            //Visit the node by Printing the node data
            //System.out.printf("%4x ",root.data);
            mPostOrd.add(root.data);
        }
    }
}
