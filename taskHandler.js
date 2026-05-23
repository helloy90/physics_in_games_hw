export class TaskHandler
{
  constructor()
  {
    this.parts = new Map();
    this.currentPart = null;
    this.currentPartName = "";
  }

  loadPart(name, part)
  {
    this.parts.set(name, part)
    part.init();
  }

  setPart(name)
  {
    const part = this.parts.get(name);
    if (!part)
    {
      return;
    }

    this.currentPart = part;
    this.currentPartName = name;
    this.currentPart.reset();
  }

  update(dt)
  {
    if (this.currentPart)
    {
      this.currentPart.update(dt);
    }
  }

  render()
  {
    if (this.currentPart)
    {
      this.currentPart.render();
    }
  }

  render2D()
  {
    if (this.currentPart)
    {
      this.currentPart.render2D();
    }
  }

  keyPressed(key)
  {
    if (this.currentPart?.keyPressed)
    {
      this.currentPart.keyPressed(key);
    }
  }

  mousePressed(x, y)
  {
    if (this.currentPart?.mousePressed)
    {
      this.currentPart.mousePressed(x, y);
    }
  }

  mouseDragged(x, y)
  {
    if (this.currentPart?.mouseDragged)
    {
      this.currentPart.mouseDragged(x, y);
    }
  }

  mouseReleased(x, y)
  {
    if (this.currentPart?.mouseReleased)
    {
      this.currentPart.mouseReleased(x, y);
    }
  }

  mouseWheel(event)
  {
    if (this.currentPart?.mouseWheel)
    {
      this.currentPart.mouseWheel(event);
    }
  }
}
