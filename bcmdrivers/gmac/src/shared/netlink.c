#include <net/netlink.h>
#include <net/genetlink.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>

#include "bcmiproc_phy5461s.h"
#include "typedefs.h"


//extern int phy5461_rd_reg(uint eth_num, uint phyaddr, uint32 flags, uint16 reg_bank, uint8 reg_addr, uint16 *data);
//extern int phy5461_wr_reg(uint eth_num, uint phyaddr, uint32 flags, uint16 reg_bank, uint8 reg_addr, uint16 *data);
static int debug=1;
#define LOGC(format, args...) if(debug == 1) printk(format, ##args)

/* netlink attributes */
enum {
     ATTR_PHY_CFG_UNSPEC,
     ATTR_PHY_CFG_DATA,
     ATTR_PHY_CFG_MAX,
};
#define DOC_EXMPL_A_MAX (ATTR_PHY_CFG_MAX - 1)

/* attribute policy */
static struct nla_policy doc_exmpl_genl_policy[DOC_EXMPL_A_MAX + 1] = {
     [ATTR_PHY_CFG_DATA] = { .type = NLA_NUL_STRING  },
};

/* commands 定义命令类型，用户空间以此来表明需要执行的命令 */
enum {
   GENL_PHY_CFG_UNSPEC,
   GENL_PHY_CFG_GET,
   GENL_PHY_CFG_SET,
   GENL_PHY_CFG_MAX,
};
#define DOC_EXMPL_C_MAX (GENL_PHY_CFG_MAX - 1)

/* family definition */
static struct genl_family doc_exmpl_genl_family = {
     .id = GENL_ID_GENERATE,   //这里不指定family ID，由内核进行分配
     .hdrsize = 0,             //自定义的头部长度，参考genl数据包结构
     .name = "DOC_EXMPL",      //这里定义family的名称，user program需要根据这个名字来找到对应的family ID。
     .version = 1,
     .maxattr = DOC_EXMPL_A_MAX,
};

typedef struct
{
	unsigned int   eth_num;   /* MII 控制器，预留     */
	unsigned int   phyaddr;   /* PHY 地址             */
	unsigned int   flags;     /* 预留                 */
	unsigned short reg_bank;  /* 寄存器的页地址，预留 */
	unsigned char  reg_addr;  /* 寄存器地址           */
	unsigned short data;      /* 寄存器的值           */
} phy_cfg_t;

static struct genl_multicast_group doc_exmpl_genl_mcgrp = {
       .name = "DOC_EXMPL_GRP",
};

static inline int genl_msg_prepare_usr_msg(u8 cmd, size_t size, pid_t pid, struct sk_buff **skbp)
{
    struct sk_buff *skb;

    /* create a new netlink msg */
    skb = genlmsg_new(size, GFP_KERNEL);
    if (skb == NULL) {
        return -ENOMEM;
    }

    /* Add a new netlink message to an skb */
    genlmsg_put(skb, pid, 0, &doc_exmpl_genl_family, 0, cmd);

    *skbp = skb;
    return 0;
}

static inline int genl_msg_mk_usr_msg(struct sk_buff *skb, int type, void *data, int len)
{
    int rc;

    /* add a netlink attribute to a socket buffer */
    if ((rc = nla_put(skb, type, len, data)) != 0) {
        return rc;
    }
    return 0;
}

/**
* genl_msg_send_to_user - 通过generic netlink发送数据到netlink
*
* @data: 发送数据缓存
* @len:  数据长度 单位：byte
* @pid:  发送到的客户端pid
*
* return:
*    0:       成功
*    -1:      失败
*/
int genl_msg_send_to_user(void *data, int len, pid_t pid)
{
    struct sk_buff *skb;
    size_t size;
    void *head;
    int rc;

    size = nla_total_size(len); /* total length of attribute including padding */

    rc = genl_msg_prepare_usr_msg(GENL_PHY_CFG_GET, size, pid, &skb);
    if (rc) {
        return rc;
    }

    rc = genl_msg_mk_usr_msg(skb, ATTR_PHY_CFG_DATA, data, len);
    if (rc) {
        kfree_skb(skb);
        return rc;
    }

    head = genlmsg_data(nlmsg_data(nlmsg_hdr(skb)));

    rc = genlmsg_end(skb, head);
    if (rc < 0) {
        kfree_skb(skb);
        return rc;
    }

    rc = genlmsg_unicast(&init_net, skb, pid);
    if (rc < 0) {
        return rc;
    }

    return 0;
}

//echo command handler, 命令处理函数，当接收到user program发出的命令后，这个函数会被内核调用
static int phy_reg_read(struct sk_buff *skb, struct genl_info *info)
{
   /* message handling code goes here; return 0 on success, negative values on failure */
   struct nlmsghdr *nlhdr;
   struct genlmsghdr *genlhdr;
   struct nlattr *nlh;
   char *str;
   int ret;
   phy_cfg_t *phy_cfg;
   
   nlhdr = nlmsg_hdr(skb);
   genlhdr = nlmsg_data(nlhdr);
   nlh = genlmsg_data(genlhdr);
   //str = nla_data(nlh);
   phy_cfg = (phy_cfg_t *)nla_data(nlh);
   
   phy5461_rd_reg(phy_cfg->eth_num, phy_cfg->phyaddr, phy_cfg->flags, 
				 phy_cfg->reg_bank, phy_cfg->reg_addr, &(phy_cfg->data));

//   printk("###kernel read ### 0x%x  0x%x  0x%x  0x%x  0x%x  0x%x \n",
//				phy_cfg->eth_num, phy_cfg->phyaddr, phy_cfg->flags, 
//				phy_cfg->reg_bank, phy_cfg->reg_addr, phy_cfg->data);
   
   ret = genl_msg_send_to_user(phy_cfg, sizeof(phy_cfg_t), nlhdr->nlmsg_pid);

   return ret;
}

//echo command handler, 命令处理函数，当接收到user program发出的命令后，这个函数会被内核调用
static int phy_reg_write(struct sk_buff *skb, struct genl_info *info)
{
   /* message handling code goes here; return 0 on success, negative values on failure */
   struct nlmsghdr *nlhdr;
   struct genlmsghdr *genlhdr;
   struct nlattr *nlh;
   char *str;
   int ret;
   phy_cfg_t *phy_cfg;
   
   nlhdr = nlmsg_hdr(skb);
   genlhdr = nlmsg_data(nlhdr);
   nlh = genlmsg_data(genlhdr);
   //str = nla_data(nlh);
   phy_cfg = (phy_cfg_t *)nla_data(nlh);
   
   phy5461_wr_reg(phy_cfg->eth_num, phy_cfg->phyaddr, phy_cfg->flags, 
				 phy_cfg->reg_bank, phy_cfg->reg_addr, &(phy_cfg->data));
   
   //printk("###kernel write ### 0x%x  0x%x  0x%x  0x%x  0x%x  0x%x \n",
//				phy_cfg->eth_num, phy_cfg->phyaddr, phy_cfg->flags, 
//				phy_cfg->reg_bank, phy_cfg->reg_addr, phy_cfg->data);
   
   ret = genl_msg_send_to_user(phy_cfg, sizeof(phy_cfg_t), nlhdr->nlmsg_pid);

   return ret;
}


/* operation definition 将命令command echo和具体的handler对应起来 */
static struct genl_ops doc_exmpl_genl_ops_echo[] = {
    {
        .cmd    = GENL_PHY_CFG_GET,
        .doit   = phy_reg_read,
        .policy = doc_exmpl_genl_policy,
        // No access control
    },
    {
        .cmd    = GENL_PHY_CFG_SET,
        .doit   = phy_reg_write,
        .policy = doc_exmpl_genl_policy,
        .flags  = GENL_ADMIN_PERM
    },
};


static int genetlink_init(void)
{
   int rc;
   int i;

   /**
    * 1. Registering A Family
    * This function doesn't exist past linux 3.12
    */
   rc = genl_register_family(&doc_exmpl_genl_family);
   if (rc != 0)
       goto err_out1;

    for (i = 0; i < ARRAY_SIZE(doc_exmpl_genl_ops_echo); ++i) {
	   rc = genl_register_ops(&doc_exmpl_genl_family, &doc_exmpl_genl_ops_echo[i]);
	   if (rc)
		   goto err_out2;
   }


   LOGC("doc_exmpl_genl_mcgrp.id=%d \n", doc_exmpl_genl_mcgrp.id);
   LOGC("genetlink_init OK \n");
   return 0;

err_out2:
	 for (i = 0; i < ARRAY_SIZE(doc_exmpl_genl_ops_echo); ++i) {
		rc = genl_unregister_ops(&doc_exmpl_genl_family, &doc_exmpl_genl_ops_echo[i]);
	}

   genl_unregister_family(&doc_exmpl_genl_family);
err_out1:
   LOGC("Error occured while inserting generic netlink example module\n");
   return rc;
}

static void genetlink_exit(void)
{
	int i;

   LOGC("Generic Netlink Example Module unloaded. \n");

	for (i = 0; i < ARRAY_SIZE(doc_exmpl_genl_ops_echo); ++i) {
	   genl_unregister_ops(&doc_exmpl_genl_family, &doc_exmpl_genl_ops_echo[i]);
   }

   genl_unregister_family(&doc_exmpl_genl_family);
}


module_init(genetlink_init);
module_exit(genetlink_exit);

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:netlink");
MODULE_AUTHOR("Lihz <Lihz@bjhuahuan.com>");
MODULE_DESCRIPTION("Huahuan netlink test");
MODULE_LICENSE("GPL");


