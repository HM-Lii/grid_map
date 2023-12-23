```mermaid
gantt
	title  割草机导航框架开发计划
	section git熟悉和gps定位包接入git管理
		grid_map包三维分支开发 :2023-12-15,3d
		git学习及熟悉 :2023-12-16,2d
		gps定位包推送 :1d
		二维分支推送 :1d
		分布式调试 :1d
		三维分支测试推送 :a1,2023-12-21,2d
	section gecao_navigation包开发测试
		主函数调整 :after a1,1d
		规划基类进抽象并debugA* :after a1,2d
		推送正式版0.9 :b1,2023-12-25,1d
		实车测试 :1d
		修改正式版1.0 :2d
		动态规划代码开发 :after b1,3d
		
	
```

---
- [x] 推送gps_localization
- [x] 推送grid_map的2d分支
- [x] 开发grid_map的3d分支
- [x] 分布式调试
- [x] 推送grid_map的3d分支
- [ ] decider主函数调整
- [ ] debug A\*
- [ ] 规划基类抽象
- [ ] 推送gecao_navigation的正式版0.9分支
- [ ] 实车测试并修改正式版0.9分支
- [ ] 离线修改并推送正式版1.0分支
- [ ] 实车测试并合并分支
