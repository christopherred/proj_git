# USB端口固定

![bb8515b410fc96d083acc36d810cd32](assets/bb8515b410fc96d083acc36d810cd32-20241022162759-kemr8fc.png)

修改.rulers文件后重新加载文件指令：

sudo udevadm control --reload-rules  
sudo udevadm trigger

执行不了上面两个代码先执行sudo service udev restart再试试

‍
